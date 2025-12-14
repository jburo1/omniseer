#pragma once

#include <cstdint>

namespace motor_driver_config
{

  // I2C address of HiWonder motor driver
  constexpr uint8_t MOT_DRIV_I2C_ADDR = 0x34;

  // register addresses
  constexpr uint8_t ADC_BAT_ADDR                    = 0x00;
  constexpr uint8_t MOTOR_TYPE_ADDR                 = 0x14;
  constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR     = 0x15;
  constexpr uint8_t MOTOR_FIXED_SPEED_ADDR          = 0x33;
  constexpr uint8_t MOTOR_ENCODER_ADDR              = 0x3C;
  constexpr uint8_t MOTOR_TYPE_JGB37_520_12V_110RPM = 3;

  // logical wheel indices
  constexpr uint8_t WHEEL_FRONT_LEFT  = 0;
  constexpr uint8_t WHEEL_FRONT_RIGHT = 1;
  constexpr uint8_t WHEEL_BACK_LEFT   = 2;
  constexpr uint8_t WHEEL_BACK_RIGHT  = 3;

  constexpr int MAX_TICKS_PER_10_MS = 50;
  constexpr int DRIVER_SPEED_LIMIT  = MAX_TICKS_PER_10_MS;

  constexpr float enc_ticks_per_rev = 44.0f * 131.0f; // 44 pulses per motor rev, 131:1 gearbox

  // control loop timing
  constexpr uint32_t control_period_us  = 10000;  // 100 Hz
  constexpr uint32_t command_timeout_us = 1000000; // 1s

  constexpr float INV_2PI = 0.159154943f;

  struct WheelMapEntry
  {
    uint8_t channel;  // 0..3 = driver motor 1..4
    int8_t  cmd_sign; // +1 or -1
    int8_t  enc_sign; // +1 or -1
  };

  constexpr WheelMapEntry wheelMap[4] = {
      /* FRONT_LEFT  */ {2, -1, +1}, // driver motor 3
      /* FRONT_RIGHT */ {0, +1, -1}, // driver motor 1
      /* BACK_LEFT   */ {3, +1, -1}, // driver motor 4
      /* BACK_RIGHT  */ {1, -1, +1}  // driver motor 2
  };

} // namespace motor_driver_config
