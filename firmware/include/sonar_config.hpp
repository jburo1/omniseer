#pragma once

#include <cstdint>

namespace sonar_config
{
  // teensy pinout
  constexpr uint8_t SONAR_TRIG_PIN = 15;
  constexpr uint8_t SONAR_ECHO_PIN = 14;

  // max echo time (~2 m) in microseconds (~(2+2)/343 * 1,000,000)
  constexpr uint32_t SONAR_MAX_ECHO_US = 12000;

  constexpr float MIN_RANGE_M   = 0.02f;      // hc-sr04 datasheet
  constexpr float MAX_RANGE_M   = 2.00f;
  constexpr float FOV = 0.26f;

  constexpr float US_TO_METERS  = 0.0001715f; // (343 m/s) / 2 / 1e6

} // namespace sonar_config
