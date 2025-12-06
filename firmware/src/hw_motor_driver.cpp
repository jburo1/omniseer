#include "hw_motor_driver.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "i2c_helpers.hpp"
#include "motor_driver_config.hpp"

using namespace motor_driver_config;

HwMotorDriver::HwMotorDriver(TwoWire& wire, uint8_t i2c_addr) : _wire{wire}, _i2c_addr{i2c_addr} {}

void HwMotorDriver::init()
{

  Serial.println("Starting motor driver life-check");

  _wire.begin();
  _wire.setClock(400000);

  delay(50);

  // Presence check
  _wire.beginTransmission(_i2c_addr);
  uint8_t err = _wire.endTransmission();
  if (err != 0)
  {
    Serial.printf("ERROR: Motor driver NOT found on I2C (err=%u)\n", err);
    while (true)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(800);
    }
  }

  Serial.println("Motor driver detected.");

  // Configure motor type
  if (!writeRegData(_wire, _i2c_addr, MOTOR_TYPE_ADDR, &MOTOR_TYPE_JGB37_520_12V_110RPM, 1))
  {
    Serial.println("ERROR: Failed to write motor type.");
  }
  else
  {
    Serial.println("Motor type configured.");
  }

  // Encoder polarity
  constexpr uint8_t polarity = 0;
  if (!writeRegData(_wire, _i2c_addr, MOTOR_ENCODER_POLARITY_ADDR, &polarity, 1))
  {
    Serial.println("ERROR: Failed to write encoder polarity.");
  }
  else
  {
    Serial.println("Encoder polarity configured.");
  }

  // Battery read
  float vbatt = read_battery_voltage();
  Serial.printf("Battery: %.2f V\n", vbatt);

  stop_wheels();

  Serial.println("Setup complete.");
}

void HwMotorDriver::set_wheel_speeds_rad_s(const WheelSpeeds& ws)
{
  const float desired_wheel_speeds[4] = {ws.fl, ws.fr, ws.rl, ws.rr};
  int8_t      driver_speed_units[4]   = {0, 0, 0, 0};

  for (uint8_t wheel_idx = 0; wheel_idx < 4; ++wheel_idx)
  {
    const auto& wheel_map         = wheelMap[wheel_idx];
    const float wheel_speed_rad_s = desired_wheel_speeds[wheel_idx] * wheel_map.cmd_sign;

    int16_t driver_units = _rad_s_to_driver_units(wheel_speed_rad_s);

    if (driver_units > DRIVER_SPEED_LIMIT)
      driver_units = DRIVER_SPEED_LIMIT;
    if (driver_units < -DRIVER_SPEED_LIMIT)
      driver_units = -DRIVER_SPEED_LIMIT;

    driver_speed_units[wheel_map.channel] = static_cast<int8_t>(driver_units);
  }

  if (!writeRegData(_wire, _i2c_addr, MOTOR_FIXED_SPEED_ADDR,
                    reinterpret_cast<const uint8_t*>(driver_speed_units),
                    sizeof(driver_speed_units)))
  {
    Serial.println("ERROR: Failed to set wheel speeds.");
  }
}

void HwMotorDriver::stop_wheels()
{
  const uint8_t zeros[4] = {0, 0, 0, 0};

  if (!writeRegData(_wire, _i2c_addr, MOTOR_FIXED_SPEED_ADDR, zeros, sizeof(zeros)))
  {
    Serial.println("ERROR: Failed to stop wheels.");
  }
}

bool HwMotorDriver::read_encoder_counts(WheelEncoderCounts& encoder_counts)
{
  // 4 registers @ 32 bits = 16 bytes
  uint8_t buf[16];

  if (!readRegData(_wire, _i2c_addr, MOTOR_ENCODER_ADDR, buf, sizeof(buf)))
  {
    return false;
  }

  encoder_counts.timestamp_us = micros();

  for (int i = 0; i < 4; ++i)
  {
    const auto& wheel_map = wheelMap[i];
    const int   base      = i * 4;
    int32_t     v         = 0;
    v |= static_cast<int32_t>(buf[base + 0]);
    v |= static_cast<int32_t>(buf[base + 1]) << 8;
    v |= static_cast<int32_t>(buf[base + 2]) << 16;
    v |= static_cast<int32_t>(buf[base + 3]) << 24;
    encoder_counts.data[i] = v * wheel_map.enc_sign;
  }
  return true;
}

float HwMotorDriver::read_battery_voltage()
{
  // 2 bytes: battery in mV = b0 + (b1 << 8)
  uint8_t raw[2] = {0, 0};

  if (!readRegData(_wire, _i2c_addr, ADC_BAT_ADDR, raw, sizeof(raw)))
  {
    Serial.println("ERROR: Failed to read battery voltage.");
    return 0.0f;
  }

  const uint16_t millivolts = static_cast<uint16_t>(raw[0]) | (static_cast<uint16_t>(raw[1]) << 8);

  return millivolts * 0.001f;
}

int16_t HwMotorDriver::_rad_s_to_driver_units(float w_rad_s) const
{
  // rad/s -> rev/s
  float rev_s = w_rad_s * INV_2PI;

  // rev/s -> encoder ticks/s
  float ticks_s = rev_s * enc_ticks_per_rev;

  // encoder ticks/s -> encoder ticks/10ms (driver units)
  float driver_units = ticks_s * 0.01f;

  return static_cast<int16_t>(lroundf(driver_units));
}
