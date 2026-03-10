/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "blackbox.h"
#include "blackbox_encoding.h"
#include "blackbox_internal.h"
#include "blackbox_io.h"
#include "blackbox_tlv.h"

#include "common/printf.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define TLV_TAG_SIZE_IN_BYTES sizeof(uint16_t)
#define TLV_SIZE_SIZE_IN_BYTES sizeof(uint16_t)
#define TLV_HEADER_SIZE (TLV_TAG_SIZE_IN_BYTES + TLV_SIZE_SIZE_IN_BYTES)

static bool reserveTlvSpace(uint16_t bytes)
{
    if (blackboxDeviceReserveBufferSpace(bytes) != BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }

    blackboxHeaderBudget -= bytes;
    return true;
}

static void writeTlvHeader(blackboxTlvTag_e tag, uint16_t size)
{
    blackboxWriteU16(tag);
    if (size == 0)  // For zero-length TLVs, we can skip writing the size since the reader can infer it from the tag, and this saves 4 bytes in the log
        return;

    blackboxWriteU16(size);
}

void blackboxTlvWriteU8(blackboxTlvTag_e tag, uint8_t value)
{
    uint16_t writeSize = sizeof(value);
    if (!reserveTlvSpace(TLV_HEADER_SIZE + writeSize))
        return; 

    writeTlvHeader(tag, writeSize);
    blackboxWrite(value);
}

void blackboxTlvWriteU16(blackboxTlvTag_e tag, uint16_t value)
{
    uint16_t writeSize = sizeof(value);
    if (!reserveTlvSpace(TLV_HEADER_SIZE + writeSize))
        return;

    writeTlvHeader(tag, writeSize);
    blackboxWriteU16(value);
}

void blackboxTlvWriteU32(blackboxTlvTag_e tag, uint32_t value)
{
    uint16_t writeSize = sizeof(value);
    if (!reserveTlvSpace(TLV_HEADER_SIZE + writeSize))
        return;

    writeTlvHeader(tag, writeSize);
    blackboxWriteU32(value);
}

void blackboxTlvWriteI8(blackboxTlvTag_e tag, int8_t value)
{
    blackboxTlvWriteU8(tag, (uint8_t)value);
}

void blackboxTlvWriteI16(blackboxTlvTag_e tag, int16_t value)
{
    blackboxTlvWriteU16(tag, (uint16_t)value);
}

void blackboxTlvWriteI32(blackboxTlvTag_e tag, int32_t value)
{
    blackboxTlvWriteU32(tag, (uint32_t)value);
}

// string is written without terminating 0-byte
void blackboxTlvWriteString(blackboxTlvTag_e tag, const char* str)
{
    if (!str)   // TODO, return if str == NULL?
        str = "";

    uint16_t length = strlen(str);
    uint16_t totalBytes = TLV_HEADER_SIZE + length; // TODO check if this fits in uint16_t?

    if (!reserveTlvSpace(totalBytes))
        return;

    writeTlvHeader(tag, length);
    blackboxWriteString(str);   // TODO optimize by writing directly to blackbox instead of first calculating length and then strlen again in blackboxWriteString?
}

bool blackboxTlvWriteFieldDefinitions(blackboxTlvTag_e tag, const blackboxFieldDefinitionSet_t *fieldSet)
{
    const char *fieldDefinitions = fieldSet->definitions;
    const bool hasConditions = fieldSet->conditionOffset >= 0;

    // On first call, calculate total payload size
    if (xmitState.u.fieldIndex == -1) {
        uint32_t payloadSize = 0;

        for (unsigned i = 0; i < fieldSet->fieldCount; i++) {
            const blackboxFieldDefinition_t *def = (const blackboxFieldDefinition_t *)(fieldDefinitions + fieldSet->definitionStride * i);

            if (!hasConditions || testBlackboxCondition(*(const uint8_t *)((const char *)def + fieldSet->conditionOffset))) {
                const size_t nameLength = strlen(def->name);

                payloadSize += nameLength + 1; // name + null terminator
                payloadSize += 1; // fieldNameIndex
                payloadSize += 1; // signed flag
                payloadSize += 1; // I predictor / SFrame predict
                payloadSize += 1; // I encoding / SFrame encode
                if (fieldSet->isDelta) {
                    payloadSize += 1; // P predictor
                    payloadSize += 1; // P encoding
                }
            }
        }

        // Write TLV header
        uint32_t headerSize = TLV_TAG_SIZE_IN_BYTES + TLV_SIZE_SIZE_IN_BYTES;
        if (!reserveTlvSpace(headerSize)) {
            return true; // Try again later
        }

        writeTlvHeader(tag, payloadSize);
        xmitState.u.fieldIndex = 0;
    }

    // Write field definitions in chunks
    const int FIELDS_PER_ITERATION = 5;
    int fieldsWritten = 0;

    for (; xmitState.u.fieldIndex < fieldSet->fieldCount && fieldsWritten < FIELDS_PER_ITERATION; xmitState.u.fieldIndex++) {
        const blackboxFieldDefinition_t *fieldDef = (const blackboxFieldDefinition_t *)(fieldDefinitions + fieldSet->definitionStride * xmitState.u.fieldIndex);

        if (!hasConditions || testBlackboxCondition(*(const uint8_t *)((const char *)fieldDef + fieldSet->conditionOffset))) {
            const size_t nameLength = strlen(fieldDef->name);
            uint32_t fieldSize = nameLength + 1 + 1 + 1 + 1 + 1 + (fieldSet->isDelta ? 2 : 0);

            if (!reserveTlvSpace(fieldSize)) {
                return true; // Try again later
            }

            // Write name (null-terminated)
            const char* name = fieldDef->name;
            while (*name) {
                blackboxWrite(*name++);
            }
            blackboxWrite(0); // null terminator

            // Write field metadata
            blackboxWrite((uint8_t)fieldDef->fieldNameIndex);
            blackboxWrite(fieldDef->arr[0]);
            blackboxWrite(fieldDef->arr[1]);
            blackboxWrite(fieldDef->arr[2]);

            if (fieldSet->isDelta) {
                blackboxWrite(fieldDef->arr[3]);
                blackboxWrite(fieldDef->arr[4]);
            }

            fieldsWritten++;
        }
    }

    // Check if we've finished all fields
    return xmitState.u.fieldIndex < fieldSet->fieldCount;
}

bool blackboxWriteSysinfo(void)
{
    char buf[128];

    switch (xmitState.headerIndex) {
        case 0:
            blackboxTlvWriteString(BB_TLV_TAG_FIRMWARE_TYPE, "Rotorflight");
            break;
        case 1:
            tfp_sprintf(buf, "%s %s (%s) %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, shortGitRevision, targetName);
            blackboxTlvWriteString(BB_TLV_TAG_FIRMWARE_REVISION, buf);
            break;
        case 2:
            tfp_sprintf(buf, "%s %s", buildDate, buildTime);
            blackboxTlvWriteString(BB_TLV_TAG_FIRMWARE_DATE, buf);
            break;
        default:
        // All done
            return true;
    }

    ++xmitState.headerIndex;

    // Not done yet, return false
    return false;
}
}
