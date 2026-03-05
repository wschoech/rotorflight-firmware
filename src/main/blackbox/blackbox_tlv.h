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

#pragma once

#include <stdint.h>
#include <stdbool.h>

// TODO some more documentation how it works etc

// TLV Header Format:
// - Type: uint16_t (tag identifier)
// - Length: uint16_t (payload size in bytes)
// - Value: variable length (Length bytes)

// TLV Tag Definitions for Blackbox Headers
typedef enum {
        BB_TLV_TAG_PRODUCT_NAME         = 0x01,
} blackboxTlvTag_e;

// TLV value type encodings
typedef enum {
    BB_TLV_TYPE_UINT8           = 0,
    BB_TLV_TYPE_UINT16          = 1,
    BB_TLV_TYPE_UINT32          = 2,
    BB_TLV_TYPE_INT8            = 3,
    BB_TLV_TYPE_INT16           = 4,
    BB_TLV_TYPE_INT32           = 5,
    BB_TLV_TYPE_STRING          = 6,
} blackboxTlvType_e;

void blackboxTlvWriteU8(blackboxTlvTag_e tag, uint8_t value);
void blackboxTlvWriteU16(blackboxTlvTag_e tag, uint16_t value);
void blackboxTlvWriteU32(blackboxTlvTag_e tag, uint32_t value);
void blackboxTlvWriteI8(blackboxTlvTag_e tag, int8_t value);
void blackboxTlvWriteI16(blackboxTlvTag_e tag, int16_t value);
void blackboxTlvWriteI32(blackboxTlvTag_e tag, int32_t value);
void blackboxTlvWriteString(blackboxTlvTag_e tag, const char *str);