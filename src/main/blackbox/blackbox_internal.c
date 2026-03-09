
#include "blackbox_internal.h"

#include "build/debug.h"
#include "common/utils.h"
#include "config/config.h"
#include "config/feature.h"
#include "drivers/adc.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "rx/rx.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"

#ifdef USE_BLACKBOX

static uint64_t blackboxConditionCache;

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_COUNT, too_many_flight_log_conditions);

bool isFieldEnabled(FlightLogFieldSelect_e field)
{
    return (blackboxConfig()->fields & BIT(field));
}

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
    case CONDITION(ALWAYS):
        return true;

    case CONDITION(COMMAND):
        return isFieldEnabled(FIELD_SELECT(COMMAND));

    case CONDITION(SETPOINT):
        return isFieldEnabled(FIELD_SELECT(SETPOINT));

    case CONDITION(MIXER):
        return isFieldEnabled(FIELD_SELECT(MIXER));

    case CONDITION(PID):
        return isFieldEnabled(FIELD_SELECT(PID));

    case CONDITION(BOOST):
        return isFieldEnabled(FIELD_SELECT(PID)) &&
            (currentPidProfile->pid[PID_PITCH].B > 0 ||
             currentPidProfile->pid[PID_ROLL].B > 0 ||
             currentPidProfile->pid[PID_YAW].B > 0);

    case CONDITION(HSI):
        return isFieldEnabled(FIELD_SELECT(PID)) &&
            (currentPidProfile->pid[PID_PITCH].O > 0 ||
             currentPidProfile->pid[PID_ROLL].O > 0);

    case CONDITION(ATTITUDE):
        return isFieldEnabled(FIELD_SELECT(ATTITUDE));

    case CONDITION(GYRAW):
        return isFieldEnabled(FIELD_SELECT(GYRAW));

    case CONDITION(GYRO):
        return isFieldEnabled(FIELD_SELECT(GYRO));

    case CONDITION(ACC):
        return sensors(SENSOR_ACC) && isFieldEnabled(FIELD_SELECT(ACC));

    case CONDITION(MAG):
#ifdef USE_MAG
        return sensors(SENSOR_MAG) && isFieldEnabled(FIELD_SELECT(MAG));
#else
        return false;
#endif

    case CONDITION(ALT):
#ifdef USE_BARO
        return sensors(SENSOR_BARO) && isFieldEnabled(FIELD_SELECT(ALT));
#else
        return false;
#endif

    case CONDITION(HEADSPEED):
        return (getMotorCount() >= 1) && isFieldEnabled(FIELD_SELECT(RPM));
    case CONDITION(TAILSPEED):
        return (getMotorCount() >= 2) && isFieldEnabled(FIELD_SELECT(RPM));
    case CONDITION(GOVERNOR):
        return (getMotorCount() >= 1) && isFieldEnabled(FIELD_SELECT(GOV));

    case CONDITION(TMCU):
        return isFieldEnabled(FIELD_SELECT(TEMP));
    case CONDITION(TESC):
        return featureIsEnabled(FEATURE_ESC_SENSOR) && (isFieldEnabled(FIELD_SELECT(TEMP)) || isFieldEnabled(FIELD_SELECT(ESC)));
    case CONDITION(TESC2):
        return featureIsEnabled(FEATURE_ESC_SENSOR) && isFieldEnabled(FIELD_SELECT(ESC2));

    case CONDITION(ESC_TELEM):
        return featureIsEnabled(FEATURE_ESC_SENSOR) && isFieldEnabled(FIELD_SELECT(ESC));
    case CONDITION(BEC_TELEM):
        return featureIsEnabled(FEATURE_ESC_SENSOR) && isFieldEnabled(FIELD_SELECT(BEC));
    case CONDITION(ESC2_TELEM):
        return featureIsEnabled(FEATURE_ESC_SENSOR) && isFieldEnabled(FIELD_SELECT(ESC2));

    case CONDITION(MOTOR_1):
        return (getMotorCount() >= 1) && isFieldEnabled(FIELD_SELECT(MOTOR));
    case CONDITION(MOTOR_2):
        return (getMotorCount() >= 2) && isFieldEnabled(FIELD_SELECT(MOTOR));
    case CONDITION(MOTOR_3):
        return (getMotorCount() >= 3) && isFieldEnabled(FIELD_SELECT(MOTOR));
    case CONDITION(MOTOR_4):
        return (getMotorCount() >= 4) && isFieldEnabled(FIELD_SELECT(MOTOR));

    case CONDITION(SERVO_1):
        return (getServoCount() >= 1) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_2):
        return (getServoCount() >= 2) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_3):
        return (getServoCount() >= 3) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_4):
        return (getServoCount() >= 4) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_5):
        return (getServoCount() >= 5) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_6):
        return (getServoCount() >= 6) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_7):
        return (getServoCount() >= 7) && isFieldEnabled(FIELD_SELECT(SERVO));
    case CONDITION(SERVO_8):
        return (getServoCount() >= 8) && isFieldEnabled(FIELD_SELECT(SERVO));

    case CONDITION(RSSI):
        return isRssiConfigured() && isFieldEnabled(FIELD_SELECT(RSSI));

    case CONDITION(VOLTAGE):
        return isBatteryVoltageConfigured() && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(CURRENT):
        return isBatteryCurrentConfigured() && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(VBEC):
        return adcIsEnabled(ADC_VBEC) && isFieldEnabled(FIELD_SELECT(VBEC));

    case CONDITION(VBUS):
        return adcIsEnabled(ADC_VBUS) && isFieldEnabled(FIELD_SELECT(VBUS));

    case CONDITION(DEBUG):
        return (debugMode != DEBUG_NONE);

    case CONDITION(NOT_EVERY_FRAME):
        return (blackboxGetRateDenom() > 1);

    case CONDITION(NEVER):
        return false;

    default:
        return false;
    }
}

void blackboxBuildConditionCache(void)
{
    blackboxConditionCache = 0;
    for (int index = 0; index <  FLIGHT_LOG_FIELD_CONDITION_COUNT; index++) {
        if (testBlackboxConditionUncached(index)) {
            blackboxConditionCache |= BITLL(index);
        }
    }
}

bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & BITLL(condition));
}

#endif // USE_BLACKBOX
