#pragma once
#include <Wire.h>
#include <cstdint>

#include "motor_driver_config.hpp"
#include "omniseer_core/types.hpp"

using namespace omniseer_core;

class HwMotorDriver
{
public:
  explicit HwMotorDriver(TwoWire& wire     = Wire1,
                         uint8_t  i2c_addr = motor_driver_config::MOT_DRIV_I2C_ADDR);

  // establish i2c connection, initialize
  void init();

  // input closed-loop wheel speed targets in rad/s (order: FL,FR,RL,RR)
  // these are then converted into pulse/10ms and then written over i2c to the driver registers
  void set_wheel_speeds_rad_s(const WheelSpeeds& ws);

  // e-stop all wheels
  void stop_wheels();

  // returns true on success
  bool read_encoder_counts(WheelEncoderCounts& encoder_counts);

  // returns 0.0f on error
  float read_battery_voltage();

private:
  TwoWire& _wire;
  uint8_t  _i2c_addr;

  // conversion from rad/s to driver units (pulses per 10ms)
  // return wide
  int16_t _rad_s_to_driver_units(float w_rad_s) const;
};
