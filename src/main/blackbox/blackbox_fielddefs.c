  /*
   * This file is part of Rotorflight.
   *
   * Rotorflight is free software. You can redistribute it and/or modify
   * it under the terms of the GNU General Public License as published by
   * the Free Software Foundation, either version 3 of the License, or
   * (at your option) any later version.
   *
   * Rotorflight is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   * See the GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with this software. If not, see <https://www.gnu.org/licenses/>.
   */

#include "blackbox_fielddefs.h"

#include <stddef.h>

#include "platform.h"

#ifdef USE_BLACKBOX

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:
#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)

#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

#define ENCODING_NULL FLIGHT_LOG_FIELD_ENCODING_NULL

STATIC_ASSERT(sizeof(blackboxDeltaFieldDefinition_t) <= UINT8_MAX, blackbox_delta_field_definition_stride_fits_in_u8);
STATIC_ASSERT(sizeof(blackboxConditionalFieldDefinition_t) <= UINT8_MAX, blackbox_conditional_field_definition_stride_fits_in_u8);
STATIC_ASSERT(sizeof(blackboxSimpleFieldDefinition_t) <= UINT8_MAX, blackbox_simple_field_definition_stride_fits_in_u8);
STATIC_ASSERT(offsetof(blackboxDeltaFieldDefinition_t, condition) <= INT16_MAX, blackbox_delta_condition_offset_fits_in_i16);
STATIC_ASSERT(offsetof(blackboxConditionalFieldDefinition_t, condition) <= INT16_MAX, blackbox_conditional_condition_offset_fits_in_i16);

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] =
{
    /* loop iteration doesn't appear in P frames since it always increments */
    {"loopIteration", -1, UNSIGNED, .Ipredict = PREDICT(0),    .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(INC),           .Pencode = ENCODING_NULL,        CONDITION(ALWAYS)},

    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(LINEAR),        .Pencode = ENCODING(SIGNED_VB),  CONDITION(ALWAYS)},

    /* RC commands are encoded together as a group in P-frames: */
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(COMMAND)},
    {"rcCommand",   4, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(COMMAND)},

    /* setpoint - define 4 fields like RC command */
    {"setpoint",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},
    {"setpoint",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},
    {"setpoint",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},
    {"setpoint",    3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(SETPOINT)},

    /* Mixer inputs */
    {"mixer",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},
    {"mixer",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},
    {"mixer",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},
    {"mixer",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(MIXER)},

    /* PID control terms */
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisF",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisF",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},
    {"axisF",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(PID)},

    /* PID FF Boost terms */
    {"axisB",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(BOOST)},
    {"axisB",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(BOOST)},
    {"axisB",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(BOOST)},

    /* HSI Offset terms */
    {"axisO",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(HSI)},
    {"axisO",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(HSI)},
    {"axisO",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(HSI)},

    /* Attitude Euler angles in 0.1deg steps */
    {"attitude",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(ATTITUDE)},
    {"attitude",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(ATTITUDE)},
    {"attitude",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32),  CONDITION(ATTITUDE)},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroRAW",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRAW)},
    {"gyroRAW",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRAW)},
    {"gyroRAW",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRAW)},

    {"gyroADC",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRO)},
    {"gyroADC",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRO)},
    {"gyroADC",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(GYRO)},

    {"accADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(ACC)},
    {"accADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(ACC)},
    {"accADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(ACC)},

#ifdef USE_MAG
    {"magADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(MAG)},
    {"magADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(MAG)},
    {"magADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(MAG)},
#endif

#ifdef USE_BARO
    {"altitude",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(ALT)},
#ifdef USE_VARIO
    {"vario",      -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(ALT)},
#endif
#endif

    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(RSSI)},

    {"Vbat",       -1, UNSIGNED, .Ipredict = PREDICT(VBATREF), .Iencode = ENCODING(NEG_14BIT),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(VOLTAGE)},
    {"Ibat",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(CURRENT)},

    {"Vbec",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(VBEC)},
    {"Vbus",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(VBUS)},

    {"EscV",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC_TELEM)},
    {"EscI",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC_TELEM)},
    {"EscCap",     -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC_TELEM)},
    {"EscRPM",     -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC_TELEM)},
    {"EscThr",     -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC_TELEM)},
    {"EscPwm",     -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC_TELEM)},

    {"BecV",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(BEC_TELEM)},
    {"BecI",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(BEC_TELEM)},

    {"Esc2V",      -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC2_TELEM)},
    {"Esc2I",      -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC2_TELEM)},
    {"Esc2Cap",    -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC2_TELEM)},
    {"Esc2RPM",    -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(ESC2_TELEM)},

    {"Tmcu",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(TMCU)},
    {"Tesc",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(TESC)},
    {"Tbec",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(TBEC)},
    {"Tesc2",      -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB),  CONDITION(TESC2)},

    {"govP",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(GOVERNOR)},
    {"govI",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(GOVERNOR)},
    {"govD",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(GOVERNOR)},
    {"govF",       -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16),  CONDITION(GOVERNOR)},
    {"govSum",     -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(GOVERNOR)},
    {"govTarget",  -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(GOVERNOR)},
    {"govRequest", -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(GOVERNOR)},

    {"headspeed",  -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(HEADSPEED)},
    {"tailspeed",  -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB),  .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(TAILSPEED)},

    {"motor",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_1)},
    {"motor",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_2)},
    {"motor",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_3)},
    {"motor",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(MOTOR_4)},

    {"servo",       0, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_1)},
    {"servo",       1, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_2)},
    {"servo",       2, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_3)},
    {"servo",       3, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_4)},
    {"servo",       4, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_5)},
    {"servo",       5, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_6)},
    {"servo",       6, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_7)},
    {"servo",       7, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB),  CONDITION(SERVO_8)},

    {"debug",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},
    {"debug",       7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),    .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB),  CONDITION(DEBUG)},

};

const blackboxFieldDefinitionSet_t blackboxMainFieldSet = {
    .definitions = blackboxMainFields,
    .fieldCount = ARRAYLEN(blackboxMainFields),
    .definitionStride = (uint8_t)sizeof(blackboxMainFields[0]),
    .conditionOffset = (int16_t)offsetof(blackboxDeltaFieldDefinition_t, condition),
    .isDelta = true,
};

#ifdef USE_GPS
// GPS position/vel frame
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

const blackboxFieldDefinitionSet_t blackboxGpsGFieldSet = {
    .definitions = blackboxGpsGFields,
    .fieldCount = ARRAYLEN(blackboxGpsGFields),
    .definitionStride = (uint8_t)sizeof(blackboxGpsGFields[0]),
    .conditionOffset = (int16_t)offsetof(blackboxConditionalFieldDefinition_t, condition),
    .isDelta = false,
};

// GPS home frame
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)}
};

const blackboxFieldDefinitionSet_t blackboxGpsHFieldSet = {
    .definitions = blackboxGpsHFields,
    .fieldCount = ARRAYLEN(blackboxGpsHFields),
    .definitionStride = (uint8_t)sizeof(blackboxGpsHFields[0]),
    .conditionOffset = -1,
    .isDelta = false,
};
#endif

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags",       -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
    {"stateFlags",            -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},

    {"failsafePhase",         -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxSignalReceived",      -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)}
};

const blackboxFieldDefinitionSet_t blackboxSlowFieldSet = {
    .definitions = blackboxSlowFields,
    .fieldCount = ARRAYLEN(blackboxSlowFields),
    .definitionStride = (uint8_t)sizeof(blackboxSlowFields[0]),
    .conditionOffset = -1,
    .isDelta = false,
};

#endif // USE_BLACKBOX
