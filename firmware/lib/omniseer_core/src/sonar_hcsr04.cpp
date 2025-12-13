#include "omniseer_core/sonar_hcsr04.hpp"

#include <limits>

namespace omniseer_core {

static SonarHcsr04* g_sonar_instance = nullptr;

SonarHcsr04::SonarHcsr04(uint8_t trig_pin,
                         uint8_t echo_pin,
                         uint32_t max_echo_us)
    : _trig_pin{trig_pin}
    , _echo_pin{echo_pin}
    , _max_echo_us{max_echo_us}
{}

void SonarHcsr04::init() {
    pinMode(_trig_pin, OUTPUT);
    digitalWrite(_trig_pin, LOW);

    pinMode(_echo_pin, INPUT);

    noInterrupts();
    g_sonar_instance   = this;
    _echo_state        = EchoState::Idle;
    _measurement_ready = false;
    _last_trigger_us   = micros();
    interrupts();

    attachInterrupt(_echo_pin, sonar_hcsr04_isr_trampoline, CHANGE);

    _latest_reading = SonarReading{
        .stamp_us = micros(),
        .range_m  = 0.0f,
        .valid    = false
    };
}

bool SonarHcsr04::trigger() {
    bool can_trigger = false;

    noInterrupts();
    if (_echo_state == EchoState::Idle) {
        _echo_state        = EchoState::WaitRise;
        _measurement_ready = false;
        _last_trigger_us   = micros();
        can_trigger        = true;
    }
    interrupts();

    if (!can_trigger) {
        return false;
    }

    // 10 us trigger pulse
    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);

    return true;
}

void SonarHcsr04::echo_handler() {
    const uint32_t now = micros();

    switch (_echo_state) {
    case EchoState::WaitRise:
        // ECHO going HIGH
        if (digitalRead(_echo_pin)) {
            _echo_rise_us = now;
            _echo_state   = EchoState::WaitFall;
        }
        break;

    case EchoState::WaitFall:
        // ECHO going LOW
        if (!digitalRead(_echo_pin)) {
            _echo_fall_us      = now;
            _measurement_ready = true;
            _echo_state        = EchoState::Idle;
        }
        break;

    case EchoState::Idle:
    default:
        break;
    }
}

void SonarHcsr04::update() {
    const uint32_t now = micros();

    // Consume completed measurement from ISR
    bool     ready = false;
    uint32_t rise  = 0;
    uint32_t fall  = 0;

    noInterrupts();
    if (_measurement_ready) {
        _measurement_ready = false;
        ready              = true;
        rise               = _echo_rise_us;
        fall               = _echo_fall_us;
    }
    interrupts();

    if (ready) {
        const uint32_t duration_us = fall - rise;

        SonarReading r;
        r.stamp_us = rise;

        if (duration_us == 0) {
            r.valid   = false;
            r.range_m = 0.0f;
        } else if (duration_us > _max_echo_us) {
            r.valid   = true;
            r.range_m = std::numeric_limits<float>::infinity();
        } else {
            r.valid   = true;
            r.range_m = duration_us * US_TO_METERS;
        }

        _latest_reading = r;
    }

    // Handle timeout: too long since trigger while not Idle
    EchoState state_snapshot;
    uint32_t  last_trig_snapshot;

    noInterrupts();
    state_snapshot     = _echo_state;
    last_trig_snapshot = _last_trigger_us;
    interrupts();

    if (state_snapshot != EchoState::Idle) {
        const uint32_t since_trig = now - last_trig_snapshot;
        if (since_trig > _max_echo_us + 1000) {

            // Give up on this measurement
            noInterrupts();
            _echo_state        = EchoState::Idle;
            _measurement_ready = false;
            interrupts();

            SonarReading r;
            r.stamp_us = now;
            r.valid    = true;
            r.range_m  = std::numeric_limits<float>::infinity();
            _latest_reading = r;
        }
    }

}

// ISR trampoline
void sonar_hcsr04_isr_trampoline() {
    if (g_sonar_instance) {
        g_sonar_instance->echo_handler();
    }
}

} //namespace omniseer_core
