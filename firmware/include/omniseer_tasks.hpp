#pragma once
#include "scheduler.hpp"

constexpr uint32_t MOTOR_CMD_US = 20000;   // 50 Hz
constexpr uint32_t ENCODERS_US  = 1000000; // 50 Hz
constexpr uint32_t BATTERY_US   = 1000000; // 1 Hz
constexpr uint32_t IMU_US       = 1000000; // 100 Hz
constexpr uint32_t SONAR_US     = 50000;   // 20 Hz
constexpr uint32_t EXECUTOR_SPIN_US     = 50000;   // 20 Hz


void init_micro_ros();
void init_peripherals();

void task_motor_cmd();
void task_encoders();
void task_battery();
void task_imu();
void task_sonar();
void task_micro_ros();

template <std::size_t N> void register_robot_tasks(Scheduler<N>& scheduler)
{
  scheduler.add_task(MOTOR_CMD_US, task_motor_cmd);
  scheduler.add_task(ENCODERS_US, task_encoders);
  scheduler.add_task(BATTERY_US, task_battery);
  scheduler.add_task(IMU_US, task_imu);
  scheduler.add_task(SONAR_US, task_sonar);
  scheduler.add_task(EXECUTOR_SPIN_US, task_micro_ros);

}
