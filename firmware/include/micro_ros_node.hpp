#pragma once

#include <cstdint>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>

#include "omniseer_core/motion_controller.hpp"
#include "omniseer_core/imu_bno055.hpp"
#include "omniseer_core/sonar_hcsr04.hpp"
#include "hw_motor_driver.hpp"

#include <micro_ros_platformio.h>

using namespace omniseer_core;

struct MicroRosConfig {
    const char* node_name      = "omniseer_teensy";
    const char* node_namespace = "";
    const char* encoder_topic     = "encoder_counts";
    const char* imu_topic      = "imu/data";
    const char* sonar_topic    = "sonar/range";
    const char* cmd_vel_topic  = "cmd_vel";
    uint32_t    spin_budget_us = 500;
};

class MicroRosNode {
public:
    MicroRosNode(MotionController& motion_controller,
                 ImuBno055&       imu,
                 SonarHcsr04&     sonar,
                 HwMotorDriver&    motor_driver,
                 const MicroRosConfig& config = {});

    // initializes the micro-ROS support library,sets up publishers and subscribers
    bool init();

    // runs the micro-ROS executor for a timeslice so callbacks execute
    void spin_executor(uint32_t budget_us);

    void publish_encoder_ticks();
    void publish_sonar();
    void publish_imu();


private:
    void _handle_cmd_vel(const geometry_msgs__msg__Twist& msg);
    void _init_transport();
    void _create_entities();

    // micro-ROS core handles
    rcl_allocator_t _allocator{};
    rclc_support_t  _support{};
    rcl_node_t      _node{};
    rclc_executor_t _executor{};

    rcl_publisher_t    _encoder_pub{};
    rcl_publisher_t    _imu_pub{};
    rcl_publisher_t    _sonar_pub{};
    rcl_subscription_t _cmd_vel_sub{};

    ??? _encoder_msg;
    sensor_msgs__msg__Imu   _imu_msg{};
    geometry_msgs__msg__Twist _cmd_vel_msg{};
    sensor_msgs__msg__Range _sonar_msg{};

    MotionController& _motion_controller;
    ImuBno055&        _imu;
    SonarHcsr04&      _sonar;
    HwMotorDriver&     _motor_driver;

    MicroRosConfig _config;
};
