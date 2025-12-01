#pragma once
#include <Arduino.h>
#include <cstdint>
#include <cstddef>

template<std::size_t N>
class Scheduler {
public:
    using TaskFn = void (*)();

    struct Task {
        uint32_t period_us = 0;
        uint32_t next_run_us = 0;
        TaskFn   fn         = nullptr;
        bool     used       = false;
    };

    void add_task(uint32_t period_us, TaskFn fn) {
        for (std::size_t i = 0; i < N; ++i) {
            if (!_tasks[i].used) {
                _tasks[i].used        = true;
                _tasks[i].period_us   = period_us;
                _tasks[i].next_run_us = micros() + period_us;
                _tasks[i].fn          = fn;
                return;
            }
        }
    }

    void tick() {
        uint32_t now = micros();
        for (std::size_t i = 0; i < N; ++i) {
            Task &t = _tasks[i];
            if (!t.used || !t.fn) continue;
            if (static_cast<int32_t>(now - t.next_run_us) >= 0) {
                t.next_run_us += t.period_us;
                t.fn();
            }
        }
    }

private:
    Task _tasks[N];
};
