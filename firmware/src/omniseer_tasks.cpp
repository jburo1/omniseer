#include "omniseer_tasks.hpp"

#include "hw_motor_driver.hpp"
#include "hw_serial_cmd.hpp"
#include "imu_config.hpp"
#include "motor_driver_config.hpp"
#include "omniseer_config.hpp"
#include "omniseer_core/imu_bno055.hpp"
#include "omniseer_core/mecanum_kinematics.hpp"
#include "omniseer_core/motion_controller.hpp"
#include "omniseer_core/sonar_hcsr04.hpp"
#include "micro_ros_node.hpp"
#include "sonar_config.hpp"

using namespace omniseer_core;
using namespace omniseer_config;
using namespace motor_driver_config;
using namespace imu_config;
using namespace sonar_config;


MecanumKinematics kinematics(wheel_radius_m, half_length_m, half_width_m);
MotionController  motion_controller(kinematics, command_timeout_us);
HwMotorDriver     motor_driver(Wire1, MOT_DRIV_I2C_ADDR);
ImuBno055         imu_bno(Wire, IMU_SENSOR_ID, IMU_I2C_ADDR);
SonarHcsr04       sonar_hcsr04(SONAR_TRIG_PIN, SONAR_ECHO_PIN, SONAR_MAX_ECHO_US);
MicroRosNode      micro_ros_node(motion_controller, imu_bno, sonar_hcsr04, motor_driver);


void init_micro_ros(){
  micro_ros_node.init();
}

void init_peripherals()
{
  pinMode(LED_BUILTIN, OUTPUT);
  motor_driver.init();
  imu_bno.init();
  sonar_hcsr04.init();
}

void task_motor_cmd()
{
  const uint32_t now_us = micros();

  CmdVel cmd_vel;
  if (hw_serial_cmd::poll(cmd_vel))
  {
    motion_controller.set_cmd_vel(cmd_vel, now_us);
  }

  WheelSpeeds ws{};
  motion_controller.update(now_us, ws);
  motor_driver.set_wheel_speeds_rad_s(ws);
}

void task_encoders()
{
  WheelEncoderCounts counts{};
  if (motor_driver.read_encoder_counts(counts))
  {
    Serial.printf("enc: FR=%ld RR=%ld FL=%ld RL=%ld time=%ld\n", (long) counts.data[0],
                  (long) counts.data[1], (long) counts.data[2], (long) counts.data[3],
                  (long) counts.timestamp_us);
  }
  else
  {
    Serial.println("ENCODER READ FAIL");
  }
}

void task_battery()
{
  float battery_voltage = motor_driver.read_battery_voltage();
  if (battery_voltage)
  {
    Serial.printf("Battery: %.2f V\n", battery_voltage);
  }
  else
  {
    Serial.println("BATTERY READ FAIL");
  }
}

void task_imu()
{
  imu_bno.update();

  const ImuSample& s = imu_bno.latest();

  Serial.printf("imu: t=%lu us "
                "q=(%.3f, %.3f, %.3f, %.3f) "
                "w=(%.3f, %.3f, %.3f) rad/s "
                "a=(%.3f, %.3f, %.3f) m/s^2 "
                "calib=(%u,%u,%u,%u)\n",
                static_cast<unsigned long>(s.t_us), s.qx, s.qy, s.qz, s.qw, s.wx, s.wy, s.wz, s.ax,
                s.ay, s.az, s.calib_sys, s.calib_gyro, s.calib_accel, s.calib_mag);
}

void task_sonar()
{
  sonar_hcsr04.update();
  sonar_hcsr04.trigger();

  SonarReading s = sonar_hcsr04.latest();
  if (s.valid)
  {
    Serial.printf("sonar: t=%lu us range=%.3f m\n", static_cast<unsigned long>(s.stamp_us),
                  s.range_m);
  }
  else
  {
    Serial.println("sonar: invalid");
  }
}

void task_micro_ros(){

}
