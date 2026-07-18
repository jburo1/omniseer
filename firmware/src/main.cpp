#include <Arduino.h>

#include "micro_ros_config.hpp"
#include "omniseer_tasks.hpp"
#include "scheduler.hpp"

Scheduler<task_count> scheduler;
IntervalTimer         main_loop_watchdog_timer;

namespace
{

  volatile uint32_t g_last_main_loop_ms = 0;

  void kick_main_loop_watchdog()
  {
    g_last_main_loop_ms = millis();
  }

  void check_main_loop_watchdog()
  {
    const uint32_t now_ms = millis();
    if ((now_ms - g_last_main_loop_ms) > micro_ros_config::MAIN_LOOP_WATCHDOG_TIMEOUT_MS)
    {
      SCB_AIRCR = 0x05FA0004;
    }
  }

} // namespace

void setup()
{
  Serial.begin(115200);

  const uint32_t serial_wait_start_ms = millis();
  while (!Serial && (millis() - serial_wait_start_ms) < micro_ros_config::SERIAL_READY_TIMEOUT_MS)
  {
    delay(10);
  }

  init_peripherals();
  init_micro_ros();
  register_robot_tasks(scheduler);
  kick_main_loop_watchdog();
  main_loop_watchdog_timer.begin(check_main_loop_watchdog,
                                 micro_ros_config::MAIN_LOOP_WATCHDOG_PERIOD_US);
}

void loop()
{
  kick_main_loop_watchdog();
  scheduler.tick();
  kick_main_loop_watchdog();
}
