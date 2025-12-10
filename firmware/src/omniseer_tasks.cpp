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
#include "omniseer_core/sonar_config.hpp"
#include "micro_ros_node.hpp"

using namespace omniseer_core;
using namespace omniseer_config;
using namespace motor_driver_config;
using namespace imu_config;
using namespace sonar_config;
using namespace micro_ros_config;

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

void task_motor_cmd(){
  WheelSpeeds ws{};
  motion_controller.update(micros(), ws);
  motor_driver.set_wheel_speeds_rad_s(ws);
}

void task_spin_executor(){
    micro_ros_node.spin_executor(SPIN_BUDGET_US);
}

void task_encoders()
{
    micro_ros_node.publish_encoder_ticks();
}

void task_battery()
{
    micro_ros_node.publish_battery();
}

void task_imu()
{
    imu_bno.update();

    micro_ros_node.publish_imu();
}

void task_sonar()
{
    sonar_hcsr04.update();
    sonar_hcsr04.trigger();

    micro_ros_node.publish_sonar();
}
