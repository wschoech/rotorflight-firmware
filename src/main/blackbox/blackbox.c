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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "blackbox_encoding.h"
#include "blackbox_fielddefs.h"
#include "blackbox_internal.h"
#include "blackbox_io.h"
#include "blackbox_tlv.h"
#include "blackbox.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/encoding.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"
#include "common/printf.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/adc.h"

#include "fc/board_info.h"
#include "fc/rc_rates.h"
#include "fc/parameter_names.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"
#include "flight/governor.h"
#include "flight/rescue.h"
#include "flight/position.h"

#include "io/beeper.h"
#include "io/gps.h"
#include "io/serial.h"

#include "pg/blackbox.h"
#include "pg/motor.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200

static const uint8_t blackboxHeaderMagic[] = {'R', 'T', 'F', 'L', 'B', 'B', 'L'};
static const uint16_t blackboxDataVersion = 3;

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_WAIT_FOR_READY,
    BLACKBOX_STATE_INITIAL_ERASE,
    BLACKBOX_STATE_PREPARE_LOG_FILE,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
    BLACKBOX_STATE_SEND_GPS_H_HEADER,
    BLACKBOX_STATE_SEND_GPS_G_HEADER,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_CACHE_FLUSH,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_FULL,
    BLACKBOX_STATE_GRACE_PERIOD,
    BLACKBOX_STATE_SHUTTING_DOWN,
    BLACKBOX_STATE_START_ERASE,
    BLACKBOX_STATE_ERASING,
    BLACKBOX_STATE_ERASED
} BlackboxState;


typedef struct blackboxMainState_s {
    uint32_t time;

    int16_t command[5];
    int16_t setpoint[4];
    int16_t mixer[4];

    int32_t axisPID_P[XYZ_AXIS_COUNT];
    int32_t axisPID_I[XYZ_AXIS_COUNT];
    int32_t axisPID_D[XYZ_AXIS_COUNT];
    int32_t axisPID_F[XYZ_AXIS_COUNT];
    int32_t axisPID_B[XYZ_AXIS_COUNT];
    int32_t axisPID_O[XYZ_AXIS_COUNT];

    int16_t attitude[XYZ_AXIS_COUNT];
    int16_t gyroRAW[XYZ_AXIS_COUNT];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t accADC[XYZ_AXIS_COUNT];
#ifdef USE_MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef USE_BARO
    int32_t altitude;
#ifdef USE_VARIO
    int16_t vario;
#endif
#endif

    uint16_t voltage;
    uint16_t current;

    uint16_t vbec;
    uint16_t vbus;

    uint16_t esc_voltage;
    uint16_t esc_current;
    int16_t  esc_temp;
    uint16_t esc_capa;
    uint16_t esc_pwm;
    uint16_t esc_thr;
    uint32_t esc_rpm;

    uint16_t esc2_voltage;
    uint16_t esc2_current;
    int16_t  esc2_temp;
    uint16_t esc2_capa;
    uint32_t esc2_rpm;

    uint16_t bec_voltage;
    uint16_t bec_current;
    int16_t  bec_temp;

    int16_t  mcu_temp;

    uint16_t rssi;

    uint16_t headspeed;
    uint16_t tailspeed;

    govLogData_t governor;

    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];

    int32_t debug[DEBUG_VALUE_COUNT];

} blackboxMainState_t;

typedef struct blackboxGpsState_s {
    int32_t GPS_home[2];
    int32_t GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s {
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()

//From rc_controls.c
extern boxBitmask_t rcModeActivationMask;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static bool blackboxStarted = false;

static uint32_t blackboxLastArmingBeep = 0;
static uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes
static uint8_t  blackboxLastGovState = 0;
static uint8_t  blackboxLastRescueState = 0;
static uint8_t  blackboxLastAirborneState = 0;

xmitState_t xmitState;

static uint32_t blackboxIteration;

uint32_t blackboxPInterval = 0;
uint32_t blackboxIInterval = 0;
uint32_t blackboxSInterval = 0;
uint32_t blackboxGInterval = 0;

static uint32_t blackboxSlowFrameSkipCounter;
static uint32_t blackboxGPSHomeFrameSkipCounter;

static bool blackboxLoggedAnyFrames;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

static blackboxGpsState_t gpsHistory;
static blackboxSlowState_t slowHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];


/**
 * Return true if it is safe to edit the Blackbox configuration.
 */
bool blackboxMayEditConfig(void)
{
    return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static bool blackboxIsLoggingEnabled(void)
{
    return blackboxConfig()->device;
}

static bool blackboxIsLoggingPaused(void)
{
    return false;
}

static void blackboxSetState(BlackboxState newState)
{
    //Perform initial setup required for the new state
    switch (newState) {
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        blackboxLoggedAnyFrames = false;
        break;
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxHeaderBudget = 0;
        xmitState.headerIndex = 0;
        xmitState.u.startTime = millis();
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        xmitState.headerIndex = 0;
        break;
    case BLACKBOX_STATE_RUNNING:
        blackboxSlowFrameSkipCounter = blackboxSInterval; //Force a slow frame to be written on the first iteration
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        xmitState.u.startTime = millis();
        break;
    default:
        ;
    }
    blackboxState = newState;
}

static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    const int motorCount = getMotorCount();
    const int servoCount = getServoCount();

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    if (testBlackboxCondition(CONDITION(COMMAND))) {
        // Write roll, pitch, yaw and collective first:
        blackboxWriteSigned16VBArray(blackboxCurrent->command, 4);

        /*
        * Write the throttle separately from the rest of the RC data as it's unsigned.
        * Throttle lies in range [0..1000]:
        */
        blackboxWriteUnsignedVB(blackboxCurrent->command[THROTTLE]);
    }

    if (testBlackboxCondition(CONDITION(SETPOINT))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->setpoint, 4);
    }

    if (testBlackboxCondition(CONDITION(MIXER))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->mixer, 4);
    }

    if (testBlackboxCondition(CONDITION(PID))) {
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_D, XYZ_AXIS_COUNT);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_F, XYZ_AXIS_COUNT);
    }

    if (testBlackboxCondition(CONDITION(BOOST))) {
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_B, XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(HSI))) {
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_O, XYZ_AXIS_COUNT);
    }

    if (testBlackboxCondition(CONDITION(ATTITUDE))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->attitude, XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(GYRAW))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroRAW, XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(GYRO))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(ACC))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->accADC, XYZ_AXIS_COUNT);
    }
#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
    }
#endif
#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(ALT))) {
        blackboxWriteSignedVB(blackboxCurrent->altitude);
#ifdef USE_VARIO
        blackboxWriteSignedVB(blackboxCurrent->vario);
#endif
    }
#endif
    if (testBlackboxCondition(CONDITION(RSSI))) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }

    if (testBlackboxCondition(CONDITION(VOLTAGE))) {
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->voltage) & 0x3FFF);
    }
    if (testBlackboxCondition(CONDITION(CURRENT))) {
        blackboxWriteUnsignedVB(blackboxCurrent->current);
    }
    if (testBlackboxCondition(CONDITION(VBEC))) {
        blackboxWriteUnsignedVB(blackboxCurrent->vbec);
    }
    if (testBlackboxCondition(CONDITION(VBUS))) {
        blackboxWriteUnsignedVB(blackboxCurrent->vbus);
    }
    if (testBlackboxCondition(CONDITION(ESC_TELEM))) {
        blackboxWriteUnsignedVB(blackboxCurrent->esc_voltage);
        blackboxWriteUnsignedVB(blackboxCurrent->esc_current);
        blackboxWriteUnsignedVB(blackboxCurrent->esc_capa);
        blackboxWriteUnsignedVB(blackboxCurrent->esc_rpm);
        blackboxWriteUnsignedVB(blackboxCurrent->esc_thr);
        blackboxWriteUnsignedVB(blackboxCurrent->esc_pwm);
    }
    if (testBlackboxCondition(CONDITION(BEC_TELEM))) {
        blackboxWriteUnsignedVB(blackboxCurrent->bec_voltage);
        blackboxWriteUnsignedVB(blackboxCurrent->bec_current);
    }
    if (testBlackboxCondition(CONDITION(ESC2_TELEM))) {
        blackboxWriteUnsignedVB(blackboxCurrent->esc2_voltage);
        blackboxWriteUnsignedVB(blackboxCurrent->esc2_current);
        blackboxWriteUnsignedVB(blackboxCurrent->esc2_capa);
        blackboxWriteUnsignedVB(blackboxCurrent->esc2_rpm);
    }

    if (testBlackboxCondition(CONDITION(TMCU))) {
        blackboxWriteSignedVB(blackboxCurrent->mcu_temp);
    }
    if (testBlackboxCondition(CONDITION(TESC))) {
        blackboxWriteSignedVB(blackboxCurrent->esc_temp);
    }
    if (testBlackboxCondition(CONDITION(TBEC))) {
        blackboxWriteSignedVB(blackboxCurrent->bec_temp);
    }
    if (testBlackboxCondition(CONDITION(TESC2))) {
        blackboxWriteSignedVB(blackboxCurrent->esc2_temp);
    }

    if (testBlackboxCondition(CONDITION(GOVERNOR))) {
        blackboxWriteSignedVBArray(blackboxCurrent->governor.pidTerms, 4);
        blackboxWriteSignedVB(blackboxCurrent->governor.pidSum);
        blackboxWriteUnsignedVB(blackboxCurrent->governor.targetHS);
        blackboxWriteUnsignedVB(blackboxCurrent->governor.requestHS);
    }

    if (testBlackboxCondition(CONDITION(HEADSPEED))) {
        blackboxWriteUnsignedVB(blackboxCurrent->headspeed);
    }
    if (testBlackboxCondition(CONDITION(TAILSPEED))) {
        blackboxWriteUnsignedVB(blackboxCurrent->tailspeed);
    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR))) {
        blackboxWriteSigned16VBArray(blackboxCurrent->motor, motorCount);
    }
    if (isFieldEnabled(FIELD_SELECT(SERVO))) {
        for (int i = 0; i < servoCount; i++) {
            blackboxWriteSignedVB(blackboxCurrent->servo[i] - 1500);
        }
    }

    if (testBlackboxCondition(CONDITION(DEBUG))) {
        blackboxWriteSignedVBArray(blackboxCurrent->debug, DEBUG_VALUE_COUNT);
    }

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t *curr  = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++) {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}

#define CALC_DELTAS(delta, next, prev, count) do {  \
    for (int i = 0; i < (count); i++)               \
        delta[i] = next[i] - prev[i];               \
} while(0)


static void writeInterframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxPrev = blackboxHistory[1];

    const int motorCount = getMotorCount();
    const int servoCount = getServoCount();

    int32_t deltas[8];

    blackboxWrite('P');

    // No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    if (testBlackboxCondition(CONDITION(COMMAND))) {
        CALC_DELTAS(deltas, blackboxCurrent->command, blackboxPrev->command, 4);
        blackboxWriteTag8_4S16(deltas);

        // Calculate throttle delta
        int32_t throttleDelta = blackboxCurrent->command[THROTTLE] - blackboxPrev->command[THROTTLE];
        blackboxWriteSignedVB(throttleDelta);
    }

    if (testBlackboxCondition(CONDITION(SETPOINT))) {
        CALC_DELTAS(deltas, blackboxCurrent->setpoint, blackboxPrev->setpoint, 4);
        blackboxWriteTag8_4S16(deltas);
    }

    if (testBlackboxCondition(CONDITION(MIXER))) {
        CALC_DELTAS(deltas, blackboxCurrent->mixer, blackboxPrev->mixer, 4);
        blackboxWriteTag8_4S16(deltas);
    }

    if (testBlackboxCondition(CONDITION(PID))) {
        CALC_DELTAS(deltas, blackboxCurrent->axisPID_P, blackboxPrev->axisPID_P, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        CALC_DELTAS(deltas, blackboxCurrent->axisPID_I, blackboxPrev->axisPID_I, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        CALC_DELTAS(deltas, blackboxCurrent->axisPID_D, blackboxPrev->axisPID_D, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);

        CALC_DELTAS(deltas, blackboxCurrent->axisPID_F, blackboxPrev->axisPID_F, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);
    }

    if (testBlackboxCondition(CONDITION(BOOST))) {
        CALC_DELTAS(deltas, blackboxCurrent->axisPID_B, blackboxPrev->axisPID_B, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);
    }

    if (testBlackboxCondition(CONDITION(HSI))) {
        CALC_DELTAS(deltas, blackboxCurrent->axisPID_O, blackboxPrev->axisPID_O, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);
    }

    if (testBlackboxCondition(CONDITION(ATTITUDE))) {
        CALC_DELTAS(deltas, blackboxCurrent->attitude, blackboxPrev->attitude, XYZ_AXIS_COUNT);
        blackboxWriteTag2_3S32(deltas);
    }

    // Since gyro and acc are noisy, base their predictions on the average of the history
    if (testBlackboxCondition(CONDITION(GYRAW))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroRAW), XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(GYRO))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC), XYZ_AXIS_COUNT);
    }
    if (testBlackboxCondition(CONDITION(ACC))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), XYZ_AXIS_COUNT);
    }

    // Check for sensors that are updated periodically (so deltas are normally zero)
    int packedFieldCount = 0;

#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG))) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            deltas[packedFieldCount++] = blackboxCurrent->magADC[i] - blackboxPrev->magADC[i];
        }
    }
#endif
#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(ALT))) {
        deltas[packedFieldCount++] = blackboxCurrent->altitude - blackboxPrev->altitude;
#ifdef USE_VARIO
        deltas[packedFieldCount++] = blackboxCurrent->vario - blackboxPrev->vario;
#endif
    }
#endif
    if (testBlackboxCondition(CONDITION(RSSI))) {
        deltas[packedFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxPrev->rssi;
    }
    blackboxWriteTag8_8SVB(deltas, packedFieldCount);

    if (testBlackboxCondition(CONDITION(VOLTAGE))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->voltage - blackboxPrev->voltage);
    }
    if (testBlackboxCondition(CONDITION(CURRENT))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->current - blackboxPrev->current);
    }

    if (testBlackboxCondition(CONDITION(VBEC))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->vbec - blackboxPrev->vbec);
    }
    if (testBlackboxCondition(CONDITION(VBUS))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->vbus - blackboxPrev->vbus);
    }

    if (testBlackboxCondition(CONDITION(ESC_TELEM))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc_voltage - blackboxPrev->esc_voltage);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc_current - blackboxPrev->esc_current);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc_capa - blackboxPrev->esc_capa);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc_rpm - blackboxPrev->esc_rpm);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc_thr - blackboxPrev->esc_thr);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc_pwm - blackboxPrev->esc_pwm);
    }
    if (testBlackboxCondition(CONDITION(BEC_TELEM))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->bec_voltage - blackboxPrev->bec_voltage);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->bec_current - blackboxPrev->bec_current);
    }
    if (testBlackboxCondition(CONDITION(ESC2_TELEM))) {
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc2_voltage - blackboxPrev->esc2_voltage);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc2_current - blackboxPrev->esc2_current);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc2_capa - blackboxPrev->esc2_capa);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->esc2_rpm - blackboxPrev->esc2_rpm);
    }

    packedFieldCount = 0;
    if (testBlackboxCondition(CONDITION(TMCU))) {
       deltas[packedFieldCount++] = blackboxCurrent->mcu_temp - blackboxPrev->mcu_temp;
    }
    if (testBlackboxCondition(CONDITION(TESC))) {
        deltas[packedFieldCount++] = blackboxCurrent->esc_temp - blackboxPrev->esc_temp;
    }
    if (testBlackboxCondition(CONDITION(TBEC))) {
        deltas[packedFieldCount++] = blackboxCurrent->bec_temp - blackboxPrev->bec_temp;
    }
    if (testBlackboxCondition(CONDITION(TESC2))) {
        deltas[packedFieldCount++] = blackboxCurrent->esc2_temp - blackboxPrev->esc2_temp;
    }
    blackboxWriteTag8_8SVB(deltas, packedFieldCount);

    if (testBlackboxCondition(CONDITION(GOVERNOR))) {
        CALC_DELTAS(deltas, blackboxCurrent->governor.pidTerms, blackboxPrev->governor.pidTerms, 4);
        blackboxWriteTag8_4S16(deltas);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->governor.pidSum - blackboxPrev->governor.pidSum);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->governor.targetHS- blackboxPrev->governor.targetHS);
        blackboxWriteSignedVB((int32_t) blackboxCurrent->governor.requestHS - blackboxPrev->governor.requestHS);
    }

    if (testBlackboxCondition(CONDITION(HEADSPEED))) {
        int32_t predictor = (blackboxHistory[1]->headspeed + blackboxHistory[2]->headspeed) / 2;
        blackboxWriteSignedVB(blackboxCurrent->headspeed - predictor);
    }
    if (testBlackboxCondition(CONDITION(TAILSPEED))) {
        int32_t predictor = (blackboxHistory[1]->tailspeed + blackboxHistory[2]->tailspeed) / 2;
        blackboxWriteSignedVB(blackboxCurrent->tailspeed - predictor);
    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor), motorCount);
    }
    if (isFieldEnabled(FIELD_SELECT(SERVO))) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, servo), servoCount);
    }

    if (testBlackboxCondition(CONDITION(DEBUG))) {
        CALC_DELTAS(deltas, blackboxCurrent->debug, blackboxPrev->debug, DEBUG_VALUE_COUNT);
        blackboxWriteSignedVBArray(deltas, DEBUG_VALUE_COUNT);
    }

    // Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(void)
{
    int32_t values[3];

    blackboxWrite('S');

    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);

    /*
     * Most of the time these three values will be able to pack into one byte for us:
     */
    values[0] = slowHistory.failsafePhase;
    values[1] = slowHistory.rxSignalReceived ? 1 : 0;
    values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
    blackboxWriteTag2_3S32(values);
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(blackboxSlowState_t *slow)
{
    memcpy(&slow->flightModeFlags, &rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slow->stateFlags = stateFlags;
    slow->failsafePhase = failsafePhase();
    slow->rxSignalReceived = rxIsReceivingSignal();
    slow->rxFlightChannelsValid = rxAreFlightChannelsValid();
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 */
static void blackboxCheckAndLogSlowFrame(void)
{
    if (blackboxSlowFrameSkipCounter >= blackboxSInterval) {
        loadSlowState(&slowHistory);
        writeSlowFrame();
        blackboxSlowFrameSkipCounter = 0;
    }
    else {
        blackboxSlowState_t newSlowState;
        loadSlowState(&newSlowState);

        // Only write a slow frame if it was different from the previous state
        if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0) {
            // Use the new state as our new history
            memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
            writeSlowFrame();
            blackboxSlowFrameSkipCounter = 0;
        }
        else {
            blackboxSlowFrameSkipCounter++;
        }
    }
}

void blackboxValidateConfig(void)
{
    // If we've chosen an unsupported device, change the device to serial
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
#endif
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
#endif
    case BLACKBOX_DEVICE_SERIAL:
        // Device supported, leave the setting alone
        break;

    default:
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    }
}

static void blackboxResetIterationTimers(void)
{
    blackboxIteration = 0;
}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
static void blackboxStart(void)
{
    blackboxValidateConfig();

    if (!blackboxDeviceOpen()) {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
        return;
    }

    blackboxStarted = true;

    memset(&gpsHistory, 0, sizeof(gpsHistory));

    blackboxHistory[0] = &blackboxHistoryRing[0];
    blackboxHistory[1] = &blackboxHistoryRing[1];
    blackboxHistory[2] = &blackboxHistoryRing[2];

    vbatReference = getBatteryVoltageSample();

    //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

    /*
     * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
     * must always agree with the logged data, the results of these tests must not change during logging. So
     * cache those now.
     */
    blackboxBuildConditionCache();
    blackboxResetIterationTimers();

    /*
     * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
     * it finally plays the beep for this arming event.
     */
    blackboxLastArmingBeep = getArmingBeepTimeMicros();
    memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags)); // record startup status

    blackboxLastGovState = getGovernorState();
    blackboxLastRescueState = getRescueState();
    blackboxLastAirborneState = isAirborne();

    blackboxSetState(BLACKBOX_STATE_WAIT_FOR_READY);
}

void blackboxCheckEnabler(timeUs_t currentTimeUs)
{
    static timeUs_t gracePeriodEnd = 0;
    if (!blackboxIsLoggingEnabled()) {
        switch (blackboxState) {
        case BLACKBOX_STATE_DISABLED:
        case BLACKBOX_STATE_STOPPED:
        case BLACKBOX_STATE_SHUTTING_DOWN:
            // We're already stopped/shutting down
            break;
        case BLACKBOX_STATE_START_ERASE:
        case BLACKBOX_STATE_ERASING:
            // Busy erasing
            break;
        case BLACKBOX_STATE_RUNNING:
            gracePeriodEnd =
                currentTimeUs + blackboxConfig()->gracePeriod * 1000000;
            blackboxSetState(BLACKBOX_STATE_GRACE_PERIOD);
            FALLTHROUGH;
        case BLACKBOX_STATE_GRACE_PERIOD:
            if (cmpTimeUs(currentTimeUs, gracePeriodEnd) < 0) {
                break;
            }
            // if grace period passed:
            FALLTHROUGH;
        case BLACKBOX_STATE_PAUSED:
            blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);
            FALLTHROUGH;
        default:
            blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
        }
    }
}

#ifdef USE_GPS
static void writeGPSHomeFrame(void)
{
    blackboxWrite('H');

    blackboxWriteSignedVB(GPS_home[0]);
    blackboxWriteSignedVB(GPS_home[1]);
    //TODO it'd be great if we could grab the GPS current time and write that too

    gpsHistory.GPS_home[0] = GPS_home[0];
    gpsHistory.GPS_home[1] = GPS_home[1];
}

static void writeGPSFrame(timeUs_t currentTimeUs)
{
    blackboxWrite('G');

    /*
     * If we're logging every frame, then a GPS frame always appears just after a frame with the
     * currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
     *
     * If we're not logging every frame, we need to store the time of this GPS frame.
     */
    if (testBlackboxCondition(CONDITION(NOT_EVERY_FRAME))) {
        // Predict the time of the last frame in the main log
        blackboxWriteUnsignedVB(currentTimeUs - blackboxHistory[1]->time);
    }

    blackboxWriteUnsignedVB(gpsSol.numSat);
    blackboxWriteSignedVB(gpsSol.llh.lat - gpsHistory.GPS_home[GPS_LATITUDE]);
    blackboxWriteSignedVB(gpsSol.llh.lon - gpsHistory.GPS_home[GPS_LONGITUDE]);
    blackboxWriteUnsignedVB(gpsSol.llh.altCm / 10); // was originally designed to transport meters in int16, but +-3276.7m is a good compromise
    blackboxWriteUnsignedVB(gpsSol.groundSpeed);
    blackboxWriteUnsignedVB(gpsSol.groundCourse);

    gpsHistory.GPS_numSat = gpsSol.numSat;
    gpsHistory.GPS_coord[GPS_LATITUDE] = gpsSol.llh.lat;
    gpsHistory.GPS_coord[GPS_LONGITUDE] = gpsSol.llh.lon;
}
#endif

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(timeUs_t currentTimeUs)
{
#ifndef UNIT_TEST
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = currentTimeUs;

    // ROLL/PITCH/YAW/COLLECTIVE
    for (int i = 0; i < 4; i++) {
        blackboxCurrent->command[i] = lrintf(rcCommand[i]);
        blackboxCurrent->setpoint[i] = lrintf(getSetpoint(i));
    }

    blackboxCurrent->command[THROTTLE] = lrintf(getThrottleCommand());

    blackboxCurrent->mixer[0] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_ROLL) * 1000);
    blackboxCurrent->mixer[1] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_PITCH) * 1000);
    blackboxCurrent->mixer[2] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_YAW) * 1000);
    blackboxCurrent->mixer[3] = lrintf(mixerGetInput(MIXER_IN_STABILIZED_COLLECTIVE) * 1000);

    const pidAxisData_t *pidData = pidGetAxisData();

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_P[i] = lrintf(pidData[i].P * 1000);
        blackboxCurrent->axisPID_I[i] = lrintf(pidData[i].I * 1000);
        blackboxCurrent->axisPID_D[i] = lrintf(pidData[i].D * 1000);
        blackboxCurrent->axisPID_F[i] = lrintf(pidData[i].F * 1000);
        blackboxCurrent->axisPID_B[i] = lrintf(pidData[i].B * 1000);
        blackboxCurrent->axisPID_O[i] = lrintf(pidData[i].O * 1000);
    }

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->attitude[i] = attitude.raw[i];
        blackboxCurrent->gyroADC[i] = lrintf(gyro.gyroADCf[i]);
        blackboxCurrent->gyroRAW[i] = lrintf(gyro.gyroADCd[i]);
#ifdef USE_ACC
        blackboxCurrent->accADC[i] = lrintf(acc.accADC[i]);
#endif
#ifdef USE_MAG
        blackboxCurrent->magADC[i] = lrintf(mag.magADC[i]);
#endif
    }

#ifdef USE_BARO
    blackboxCurrent->altitude = getEstimatedAltitudeCm();
#ifdef USE_VARIO
    blackboxCurrent->vario = getEstimatedVarioCms();
#endif
#endif

    blackboxCurrent->rssi = getRssi();

    blackboxCurrent->voltage = getBatteryVoltage();
    blackboxCurrent->current = getBatteryCurrent();

    voltageMeter_t meter;
    voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BEC, &meter);
    blackboxCurrent->vbec = meter.voltage / 10;
    voltageSensorADCRead(VOLTAGE_SENSOR_ADC_BUS, &meter);
    blackboxCurrent->vbus = meter.voltage / 10;

    blackboxCurrent->mcu_temp = getCoreTemperatureCelsius();

#ifdef USE_ESC_SENSOR
    escSensorData_t *escData = getEscSensorData(0);
    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        blackboxCurrent->esc_voltage = escData->voltage / 10;
        blackboxCurrent->esc_current = escData->current / 10;
        blackboxCurrent->esc_capa = escData->consumption;
        blackboxCurrent->esc_temp = escData->temperature / 10;
        blackboxCurrent->esc_thr = escData->throttle;
        blackboxCurrent->esc_pwm = escData->pwm;
        blackboxCurrent->esc_rpm = escData->erpm;
        blackboxCurrent->bec_temp = escData->temperature2 / 10;
        blackboxCurrent->bec_voltage = escData->bec_voltage;
        blackboxCurrent->bec_current = escData->bec_current;
    }
    else {
        blackboxCurrent->esc_voltage = 0;
        blackboxCurrent->esc_current = 0;
        blackboxCurrent->esc_capa = 0;
        blackboxCurrent->esc_temp = 0;
        blackboxCurrent->esc_thr = 0;
        blackboxCurrent->esc_pwm = 0;
        blackboxCurrent->esc_rpm = 0;
        blackboxCurrent->bec_temp = 0;
        blackboxCurrent->bec_voltage = 0;
        blackboxCurrent->bec_current = 0;
    }

    escData = getEscSensorData(1);
    if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
        blackboxCurrent->esc2_voltage = escData->voltage / 10;
        blackboxCurrent->esc2_current = escData->current / 10;
        blackboxCurrent->esc2_capa = escData->consumption;
        blackboxCurrent->esc2_temp = escData->temperature / 10;
        blackboxCurrent->esc2_rpm = escData->erpm;
    }
    else {
        blackboxCurrent->esc2_voltage = 0;
        blackboxCurrent->esc2_current = 0;
        blackboxCurrent->esc2_capa = 0;
        blackboxCurrent->esc2_temp = 0;
        blackboxCurrent->esc2_rpm = 0;
    }
#endif

    blackboxCurrent->headspeed = getHeadSpeed();
    blackboxCurrent->tailspeed = getTailSpeed();

    getGovernorLogData(&blackboxCurrent->governor);

    for (int i = 0; i < getMotorCount(); i++) {
        blackboxCurrent->motor[i] = getMotorOutput(i);
    }

    for (int i = 0; i < getServoCount(); i++) {
        blackboxCurrent->servo[i] = getServoOutput(i);
    }

    for (int i = 0; i < DEBUG_VALUE_COUNT; i++) {
        blackboxCurrent->debug[i] = debug[i];
    }

#else
    UNUSED(currentTimeUs);
#endif // UNIT_TEST
}

/**
 * Write the given event to the log immediately
 */
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data)
{
    uint8_t length;

    // Only allow events to be logged after headers have been written
    if (!(blackboxState == BLACKBOX_STATE_RUNNING ||
          blackboxState == BLACKBOX_STATE_PAUSED ||
          blackboxState == BLACKBOX_STATE_GRACE_PERIOD)) {
        return;
    }

    //Shared header for event frames
    blackboxWrite('E');
    blackboxWrite(event);

    //Now serialize the data for this specific frame type
    switch (event) {
    case FLIGHT_LOG_EVENT_SYNC_BEEP:
        blackboxWriteUnsignedVB(data->syncBeep.time);
        break;
    case FLIGHT_LOG_EVENT_FLIGHTMODE:
        blackboxWriteUnsignedVB(data->flightMode.flags);
        blackboxWriteUnsignedVB(data->flightMode.lastFlags);
        break;
    case FLIGHT_LOG_EVENT_GOVSTATE:
        blackboxWriteUnsignedVB(data->govState.govState);
        break;
    case FLIGHT_LOG_EVENT_RESCUE_STATE:
        blackboxWriteUnsignedVB(data->rescueState.rescueState);
        break;
    case FLIGHT_LOG_EVENT_AIRBORNE_STATE:
        blackboxWriteUnsignedVB(data->airborneState.airborneState);
        break;
    case FLIGHT_LOG_EVENT_DISARM:
        blackboxWriteUnsignedVB(data->disarm.reason);
        break;
    case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
        if (data->inflightAdjustment.floatFlag) {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
            blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
        } else {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction);
            blackboxWriteSignedVB(data->inflightAdjustment.newValue);
        }
        break;
    case FLIGHT_LOG_EVENT_CUSTOM_DATA:
        blackboxWrite(data->data.length);
        for (int i = 0; i < data->data.length; i++)
            blackboxWrite(data->data.buffer[i]);
        break;
    case FLIGHT_LOG_EVENT_CUSTOM_STRING:
        length = strlen(data->string.buffer);
        blackboxWrite(length);
        for (int i = 0; i < length; i++)
            blackboxWrite(data->string.buffer[i]);
        break;
    case FLIGHT_LOG_EVENT_LOGGING_RESUME:
        blackboxWriteUnsignedVB(data->loggingResume.logIteration);
        blackboxWriteUnsignedVB(data->loggingResume.currentTime);
        break;
    case FLIGHT_LOG_EVENT_LOG_END:
        blackboxWriteString("End of log");
        blackboxWrite(0);
        break;
    default:
        break;
    }
}

void blackboxLogCustomData(const uint8_t *ptr, size_t length)
{
    flightLogEvent_customData_t eventData;

    eventData.buffer = ptr;
    eventData.length = length;

    blackboxLogEvent(FLIGHT_LOG_EVENT_CUSTOM_DATA, (flightLogEventData_t *)&eventData);
}

void blackboxLogCustomString(const char *ptr)
{
    flightLogEvent_customString_t eventData;

    eventData.buffer = ptr;

    blackboxLogEvent(FLIGHT_LOG_EVENT_CUSTOM_STRING, (flightLogEventData_t *)&eventData);
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        flightLogEvent_syncBeep_t eventData;
        eventData.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *)&eventData);
    }
}

/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode(void)
{
    if (memcmp(&rcModeActivationMask, &blackboxLastFlightModeFlags, sizeof(blackboxLastFlightModeFlags))) {
        flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));
        memcpy(&eventData.flags, &rcModeActivationMask, sizeof(eventData.flags));
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, (flightLogEventData_t *)&eventData);
    }

    if (getGovernorState() != blackboxLastGovState) {
        blackboxLastGovState = getGovernorState();
        flightLogEvent_govState_t eventData;
        eventData.govState = blackboxLastGovState;
        blackboxLogEvent(FLIGHT_LOG_EVENT_GOVSTATE, (flightLogEventData_t *)&eventData);
    }

    if (getRescueState() != blackboxLastRescueState) {
        blackboxLastRescueState = getRescueState();
        flightLogEvent_rescueState_t eventData;
        eventData.rescueState = blackboxLastRescueState;
        blackboxLogEvent(FLIGHT_LOG_EVENT_RESCUE_STATE, (flightLogEventData_t *)&eventData);
    }

    if (isAirborne() != blackboxLastAirborneState) {
        blackboxLastAirborneState = isAirborne();
        flightLogEvent_airborneState_t eventData;
        eventData.airborneState = blackboxLastAirborneState;
        blackboxLogEvent(FLIGHT_LOG_EVENT_AIRBORNE_STATE, (flightLogEventData_t *)&eventData);
    }
}

static bool blackboxShouldLogFastFrame(void)
{
    return (blackboxIteration % blackboxPInterval) == 0;
}

static bool blackboxShouldLogIFrame(void)
{
    return (blackboxIteration % blackboxIInterval) == 0;
}

/*
 * If the GPS home point has been updated, write the GPS home position.
 *
 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
 * still be interpreted correctly.
 *
 * Synchronise the GPS frames between the I-frames.
 */
#ifdef USE_GPS
static bool blackboxShouldLogGPSFrame(void)
{
    return (blackboxIteration % blackboxIInterval) == (blackboxIInterval / 2);
}

static bool blackboxShouldLogGpsCoordFrame(void)
{
    if (gpsSol.numSat != gpsHistory.GPS_numSat ||
        gpsSol.llh.lat != gpsHistory.GPS_coord[GPS_LATITUDE] ||
        gpsSol.llh.lon != gpsHistory.GPS_coord[GPS_LONGITUDE]) {
        return true;
    }

    return false;
}

static bool blackboxShouldLogGpsHomeFrame(void)
{
    if (GPS_home[0] != gpsHistory.GPS_home[0] ||
        GPS_home[1] != gpsHistory.GPS_home[1] ||
        blackboxGPSHomeFrameSkipCounter >= blackboxGInterval) {
        blackboxGPSHomeFrameSkipCounter = 0;
        return true;
    }
    else {
        blackboxGPSHomeFrameSkipCounter++;
    }

    return false;
}


#endif // GPS

// Called once every FC loop in PAUSED and RUNNING states
static void blackboxAdvanceIterationTimers(void)
{
    blackboxIteration++;
}

// Called once every FC loop in order to log the current state
static void blackboxLogIteration(timeUs_t currentTimeUs)
{
    if (blackboxShouldLogFastFrame()) {
        blackboxCheckAndLogArmingBeep();
        blackboxCheckAndLogFlightMode();
        blackboxCheckAndLogSlowFrame();

        loadMainState(currentTimeUs);

        if (blackboxShouldLogIFrame())
            writeIntraframe();
        else
            writeInterframe();
    }

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS) && isFieldEnabled(FIELD_SELECT(GPS))) {
        if (blackboxShouldLogGPSFrame()) {
            if (blackboxShouldLogGpsHomeFrame()) {
                writeGPSHomeFrame();
                writeGPSFrame(currentTimeUs);
            } else if (blackboxShouldLogGpsCoordFrame()) {
                writeGPSFrame(currentTimeUs);
            }
        }
    }
#endif
}

void blackboxErase(void)
{
#ifdef USE_FLASHFS
    if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH) {
        blackboxSetState(BLACKBOX_STATE_START_ERASE);
    }
#endif
}

bool isBlackboxErased(void)
{
    return isBlackboxDeviceReady();
}

void blackboxInitialErase(void)
{
#ifdef USE_FLASHFS
    if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH) {
        blackboxDeviceInitialErase();
    }
#endif
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void blackboxUpdate(timeUs_t currentTimeUs)
{
    static BlackboxState cacheFlushNextState;

    blackboxCheckEnabler(currentTimeUs);

    if (IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE) &&
        blackboxState > BLACKBOX_STATE_DISABLED && blackboxState < BLACKBOX_STATE_START_ERASE) {
        blackboxErase();
    }

    switch (blackboxState) {
    case BLACKBOX_STATE_STOPPED:
        if (blackboxIsLoggingEnabled()) {
            blackboxOpen();
            blackboxStart();
        }
        break;
    case BLACKBOX_STATE_WAIT_FOR_READY:
        if (isBlackboxDeviceReady()) {
            blackboxInitialErase();
            blackboxSetState(BLACKBOX_STATE_INITIAL_ERASE);
        }
        break;
    case BLACKBOX_STATE_INITIAL_ERASE:
        if (isBlackboxDeviceReady()) {
            blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
        }
        break;
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        if (blackboxDeviceBeginLog()) {
            blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

        /*
         * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
         * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
         */
        if (millis() > xmitState.u.startTime + 100) {
            if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
                const uint32_t magicSize = sizeof(blackboxHeaderMagic);
                const uint32_t totalHeaderSize = magicSize + sizeof(blackboxDataVersion);

                for (int i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && xmitState.headerIndex < totalHeaderSize; i++, xmitState.headerIndex++) {
                    if (xmitState.headerIndex < magicSize) {
                        blackboxWrite(blackboxHeaderMagic[xmitState.headerIndex]);
                    } else {
                        const uint32_t versionByteIndex = xmitState.headerIndex - magicSize;
                        blackboxWrite((blackboxDataVersion >> (versionByteIndex * 8)) & 0xFF);
                    }
                    blackboxHeaderBudget--;
                }
                if (xmitState.headerIndex >= totalHeaderSize) {
                    blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                }
            }
        }
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!blackboxTlvWriteFieldDefinitions(BB_TLV_TAG_FIELD_DEF_MAIN, &blackboxMainFieldSet)) {
#ifdef USE_GPS
            if (featureIsEnabled(FEATURE_GPS) && isFieldEnabled(FIELD_SELECT(GPS))) {
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADER);
            } else
#endif
                blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#ifdef USE_GPS
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!blackboxTlvWriteFieldDefinitions(BB_TLV_TAG_FIELD_DEF_GPS_H, &blackboxGpsHFieldSet) && isFieldEnabled(FIELD_SELECT(GPS))) {
            blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!blackboxTlvWriteFieldDefinitions(BB_TLV_TAG_FIELD_DEF_GPS_G, &blackboxGpsGFieldSet) && isFieldEnabled(FIELD_SELECT(GPS))) {
            blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#endif
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!blackboxTlvWriteFieldDefinitions(BB_TLV_TAG_FIELD_DEF_SLOW, &blackboxSlowFieldSet)) {
            cacheFlushNextState = BLACKBOX_STATE_SEND_SYSINFO;
            blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
        }
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0

        //Keep writing chunks of the system info headers until it returns true to signal completion
        if (blackboxWriteSysinfo()) {
            blackboxTlvWriteEndMarker();    // All headers logged, write end-of-headers marker

            /*
             * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
             * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
             * could wipe out the end of the header if we weren't careful)
             */
            cacheFlushNextState = BLACKBOX_STATE_RUNNING;
            blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
        }
        break;
    case BLACKBOX_STATE_CACHE_FLUSH:
        // Flush the cache and wait until all possible entries have been written to the media
        if (blackboxDeviceFlushForceComplete()) {
            blackboxSetState(cacheFlushNextState);
        }
        break;
    case BLACKBOX_STATE_PAUSED:
        // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
        if (!blackboxIsLoggingPaused() && blackboxShouldLogIFrame()) {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            flightLogEvent_loggingResume_t resume;

            resume.logIteration = blackboxIteration;
            resume.currentTime = currentTimeUs;

            blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
            blackboxSetState(BLACKBOX_STATE_RUNNING);

            blackboxLogIteration(currentTimeUs);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_GRACE_PERIOD:
        if (blackboxIsLoggingEnabled()) {
            blackboxSetState(BLACKBOX_STATE_RUNNING);
        }
        FALLTHROUGH;  // Keep logging during the grace period.
    case BLACKBOX_STATE_RUNNING:
        // On entry to this state, blackboxIteration reset to 0
        if (blackboxIsLoggingPaused()) {
            blackboxSetState(BLACKBOX_STATE_PAUSED);
        } else {
            blackboxLogIteration(currentTimeUs);
        }
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        //On entry of this state, startTime is set
        /*
         * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
         * since releasing the port clears the Tx buffer.
         *
         * Don't wait longer than it could possibly take if something funky happens.
         */
        if (blackboxDeviceEndLog(blackboxLoggedAnyFrames) && (millis() > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlushForce())) {
            blackboxDeviceClose();
            blackboxSetState(BLACKBOX_STATE_STOPPED);

            blackboxStarted = false;
        }
        break;
#ifdef USE_FLASHFS
    case BLACKBOX_STATE_START_ERASE:
        if (isBlackboxDeviceReady()) {
            blackboxDeviceErase();
            blackboxSetState(BLACKBOX_STATE_ERASING);
            beeper(BEEPER_BLACKBOX_ERASE);
        }
        break;
    case BLACKBOX_STATE_ERASING:
        if (isBlackboxDeviceReady()) {
            blackboxSetState(BLACKBOX_STATE_ERASED);
            beeper(BEEPER_BLACKBOX_ERASE);
        }
        break;
    case BLACKBOX_STATE_ERASED:
        if (!IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxDeviceClose();
            blackboxSetState(BLACKBOX_STATE_STOPPED);
            blackboxStarted = false;
        }
        break;
    case BLACKBOX_STATE_FULL:
        if (!blackboxIsLoggingEnabled()) {
            blackboxDeviceClose();
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
        break;
#endif
    default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (isBlackboxDeviceFull() && !blackboxConfig()->rollingErase) {
        if (blackboxState == BLACKBOX_STATE_RUNNING) {
            blackboxSetState(BLACKBOX_STATE_FULL);
        }
    }
}

uint8_t blackboxGetRateDenom(void)
{
    return blackboxPInterval;
}

void blackboxFlush(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Flush every iteration so that our runtime variance is minimized
    blackboxDeviceFlush();
}

/**
 * Call during system startup to initialize the blackbox.
 */
void blackboxInit(void)
{
    blackboxResetIterationTimers();

    blackboxPInterval = constrain(blackboxConfig()->denom, 1, 8000);

    // I-frame is written at least every 32ms or 64 P-frames
    uint32_t Imul = (32 * gyro.targetRateHz) / (1000 * blackboxPInterval);

    // Make sure Iinterval is a multiple of Pinterval
    if (Imul > 64)
        blackboxIInterval = blackboxPInterval * 64;
    else if (Imul > 0)
        blackboxIInterval = blackboxPInterval * Imul;
    else
        blackboxIInterval = blackboxPInterval;

    // S-frame is written at least every 5s
    blackboxSInterval = 5 * gyro.targetRateHz / blackboxPInterval;

    // GPS frame is written at least every 10s
    blackboxGInterval = 10 * gyro.targetRateHz / blackboxIInterval;

    if (blackboxConfig()->device)
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    else
        blackboxSetState(BLACKBOX_STATE_DISABLED);
}
#endif
