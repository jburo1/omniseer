#pragma once

#include <Arduino.h>
#include <cstdint>
#include <omniseer_core/sonar_config.hpp>

using namespace sonar_config;

namespace omniseer_core {

struct SonarReading {
    uint32_t stamp_us = 0;
    float    range_m  = 0.0f;
    bool     valid    = false;
};

class SonarHcsr04 {
public:

    SonarHcsr04(uint8_t trig_pin,
                uint8_t echo_pin,
                uint32_t max_echo_us);

    void init();

    // non-blocking: sends a pulse only if idle; returns true if triggered
    bool trigger();

    // consume ISR data, handle timeouts, and refresh latest reading
    void update();

    SonarReading latest() const { return _latest_reading; }
    float        max_range_m() const { return _max_echo_us * US_TO_METERS; }
    float        min_range_m() const { return MIN_RANGE_M; }
    float        fov() const {return FOV;}

    void echo_handler();

private:
    enum class EchoState : uint8_t { Idle, WaitRise, WaitFall };

    const uint8_t  _trig_pin;
    const uint8_t  _echo_pin;
    const uint32_t _max_echo_us;

    // ISR shared state
    volatile EchoState _echo_state       = EchoState::Idle;
    volatile uint32_t  _echo_rise_us     = 0;
    volatile uint32_t  _echo_fall_us     = 0;
    volatile bool      _measurement_ready = false;
    volatile uint32_t  _last_trigger_us  = 0;

    SonarReading _latest_reading{};
};

// trampoline for attachInterrupt
void sonar_hcsr04_isr_trampoline();

} // namespace omniseer_core
