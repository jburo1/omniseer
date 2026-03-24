#pragma once
#include <cstdint>
#include <cstddef>

namespace micro_ros_config {

    constexpr const char* NODE_NAME         = "omniseer_teensy";
    constexpr const char* NODE_NAMESPACE    = "";
    constexpr const char* ENCODER_TOPIC     = "/encoder_counts";
    constexpr const char* IMU_TOPIC         = "/imu";
    constexpr const char* SONAR_TOPIC       = "/range";
    constexpr const char* CMD_VEL_TOPIC     = "/mecanum_drive_controller/reference";
    constexpr const char* DEBUG_TOPIC       = "/micro_ros/debug";
    constexpr const char* BATTERY_TOPIC     = "/battery";

    constexpr size_t DOMAIN_ID    = 42;

    constexpr uint32_t    SPIN_BUDGET_US = 500;
    constexpr uint32_t    INIT_TIMEOUT_MS = 1000;
    constexpr uint32_t    INIT_RETRY_PERIOD_MS = 1000;
    constexpr uint32_t    SERIAL_READY_TIMEOUT_MS = 1000;
    constexpr uint32_t    AGENT_PING_PERIOD_MS   = 1000;
    constexpr uint32_t    AGENT_PING_TIMEOUT_MS  = 50;
    constexpr uint8_t     AGENT_PING_ATTEMPTS    = 1;
    constexpr uint8_t     RECONNECT_RESET_ATTEMPTS = 5;
    constexpr uint32_t    RESYNC_PERIOD_MS       = 30000;
    constexpr uint32_t    RESYNC_TIMEOUT_MS      = 50;
    constexpr size_t      DEBUG_BUF_SIZE = 128;

} // namespace micro_ros_config
