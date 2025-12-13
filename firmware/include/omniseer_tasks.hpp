#pragma once
#include "scheduler.hpp"

constexpr uint32_t MOTOR_CMD_US = 20000;   // 50 Hz
constexpr uint32_t ENCODERS_US  = 1000000; // 50 Hz
constexpr uint32_t BATTERY_US   = 1000000; // 1 Hz
constexpr uint32_t IMU_US       = 1000000; // 100 Hz
constexpr uint32_t SONAR_US     = 50000;   // 20 Hz
constexpr uint32_t EXECUTOR_SPIN_US     = 500000;   // 200 Hz

struct TaskFrequency {
  void (*fn)();
  uint32_t period_us;
};

void init_micro_ros();
void init_peripherals();

void task_motor_cmd();
void task_encoders();
void task_battery();
void task_imu();
void task_sonar();
void task_spin_executor();

constexpr TaskFrequency tasks[] = {
  {task_motor_cmd, MOTOR_CMD_US},
  {task_encoders, ENCODERS_US},
  {task_battery, BATTERY_US},
  {task_imu, IMU_US},
  {task_sonar, SONAR_US},
  {task_spin_executor, EXECUTOR_SPIN_US}
};

constexpr size_t task_count = std::size(tasks);

template <std::size_t N> void register_robot_tasks(Scheduler<N>& scheduler)
{
  for (const auto& t : tasks) {
    scheduler.add_task(t.period_us, t.fn);
  }
}
