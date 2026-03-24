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

namespace
{

  bool     g_micro_ros_ready         = false;
  bool     g_micro_ros_connected_once = false;
  uint32_t g_last_micro_ros_retry_ms = 0;
  uint8_t  g_reconnect_failures      = 0;

  void reboot_mcu_for_micro_ros_recovery()
  {
    motion_controller.set_cmd_vel(CmdVel{0.0f, 0.0f, 0.0f}, micros());
    motor_driver.stop_wheels();
    delay(50);
    SCB_AIRCR = 0x05FA0004;
    while (true)
    {
    }
  }

} // namespace


void init_micro_ros(){
  g_micro_ros_ready = micro_ros_node.init();
  if (g_micro_ros_ready)
  {
    g_micro_ros_connected_once = true;
    g_reconnect_failures = 0;
  }
  g_last_micro_ros_retry_ms = millis();
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
    if (!g_micro_ros_ready)
    {
      const uint32_t now_ms = millis();
      if ((now_ms - g_last_micro_ros_retry_ms) >= INIT_RETRY_PERIOD_MS)
      {
        g_last_micro_ros_retry_ms = now_ms;
        g_micro_ros_ready = micro_ros_node.init();
        if (g_micro_ros_ready)
        {
          g_micro_ros_connected_once = true;
          g_reconnect_failures = 0;
        }
        else if (g_micro_ros_connected_once)
        {
          ++g_reconnect_failures;
          if (g_reconnect_failures >= RECONNECT_RESET_ATTEMPTS)
          {
            reboot_mcu_for_micro_ros_recovery();
          }
        }
      }
      return;
    }

    micro_ros_node.spin_executor(SPIN_BUDGET_US);
    if (!micro_ros_node.is_ready())
    {
      g_micro_ros_ready = false;
      g_reconnect_failures = 0;
      g_last_micro_ros_retry_ms = millis();
    }
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
