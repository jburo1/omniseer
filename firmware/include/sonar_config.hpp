#pragma once

#include <cstdint>

namespace sonar_config
{

  // Teensy pinout
  constexpr uint8_t SONAR_TRIG_PIN = 15;
  constexpr uint8_t SONAR_ECHO_PIN = 14;

  // Max echo time (~2 m) in microseconds (~(2+2)/343 * 1,000,000)
  constexpr uint32_t SONAR_MAX_ECHO_US = 12000;

} // namespace sonar_config
