#include <Arduino.h>

#include "omniseer_tasks.hpp"
#include "scheduler.hpp"

Scheduler<task_count> scheduler;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  { /* wait */
  }
  init_peripherals();
  init_micro_ros();
  register_robot_tasks(scheduler);
}

void loop()
{
  scheduler.tick();
}
