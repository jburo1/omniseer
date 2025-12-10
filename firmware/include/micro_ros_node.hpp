#pragma once

#include <cstdint>
#include <cstdarg>
#include <cstdio>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/init_options.h>

#include <micro_ros_platformio.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/battery_state.h>
#include <omniseer_msgs/msg/wheel_encoder_counts.h>
#include <std_msgs/msg/string.h>

#include "micro_ros_config.hpp"
#include "omniseer_core/motion_controller.hpp"
#include "omniseer_core/imu_bno055.hpp"
#include "omniseer_core/sonar_hcsr04.hpp"
#include "hw_motor_driver.hpp"

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>

using namespace omniseer_core;
using namespace micro_ros_config;

class MicroRosNode {
public:
    MicroRosNode(MotionController& motion_controller,
                 ImuBno055&       imu,
                 SonarHcsr04&     sonar,
                 HwMotorDriver&    motor_driver);

    // initializes the micro-ROS support library,sets up publishers and subscribers
    bool init();

    // runs the micro-ROS executor for a timeslice so callbacks execute
    void spin_executor(uint32_t budget_us);

    // publishing functions
    void publish_encoder_ticks();
    void publish_sonar();
    void publish_imu();
    void publish_battery();

    // debugging
    void log_debugf(const char* fmt, ...);

    static void cmd_vel_callback(const void* msg_in);

private:
    void _init_transport();
    bool _init_core();
    void _init_messaging();

    void _sync_time(int timeout_ms);

    void _create_entities();

    void _handle_cmd_vel(const geometry_msgs__msg__Twist& msg);

    void _set_stamp_from_local_us(builtin_interfaces__msg__Time& t, uint64_t local_us);

    // micro-ROS core handles
    rcl_allocator_t _allocator{};
    rclc_support_t  _support{};
    rcl_node_t      _node{};
    rclc_executor_t _executor{};

    // publishers/subscribers
    rcl_publisher_t    _encoder_pub{};
    rcl_publisher_t    _imu_pub{};
    rcl_publisher_t    _sonar_pub{};
    rcl_publisher_t    _battery_pub{};
    rcl_publisher_t _debug_pub{};
    rcl_subscription_t _cmd_vel_sub{};

    // pooled memory for messages
    omniseer_msgs__msg__WheelEncoderCounts _encoder_msg{};
    sensor_msgs__msg__Imu   _imu_msg{};
    geometry_msgs__msg__Twist _cmd_vel_msg{};
    sensor_msgs__msg__Range _sonar_msg{};
    sensor_msgs__msg__BatteryState _battery_msg{};
    std_msgs__msg__String _debug_msg{};
    char _debug_buf[DEBUG_BUF_SIZE];

    // component references
    MotionController& _motion_controller;
    ImuBno055&        _imu;
    SonarHcsr04&      _sonar;
    HwMotorDriver&     _motor_driver;

    // time sync state
    volatile bool     _time_synced{false};
    volatile int64_t  _agent_offset_ns{0};
    uint32_t          _last_resync_ms{0};

    // debug state
    bool _debug_ready{false};
};
