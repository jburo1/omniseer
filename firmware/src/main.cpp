#include <Arduino.h>

#include "omniseer_tasks.hpp"
#include "scheduler.hpp"

Scheduler<5> scheduler;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  { /* wait */
  }
  init_peripherals();
  register_robot_tasks(scheduler);
}

void loop()
{
  scheduler.tick();
}
