#include <Arduino.h>

#include "micro_ros_config.hpp"
#include "omniseer_tasks.hpp"
#include "scheduler.hpp"

Scheduler<task_count> scheduler;

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
}

void loop()
{
  scheduler.tick();
}
