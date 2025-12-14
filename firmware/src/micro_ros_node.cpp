// https://github.com/micro-ROS/micro-ROS-demos/blob/kilted/rclc/int32_publisher/main.c

#include "micro_ros_node.hpp"

static MicroRosNode* g_micro_ros_node_instance = nullptr;

// rcl error checkers
namespace
{

  inline bool rcl_check(rcl_ret_t rc, const char* where)
  {
    if (rc != RCL_RET_OK && g_micro_ros_node_instance)
    {
      g_micro_ros_node_instance->log_debugf("RCCHECK failed at %s rc=%d", where, (int) rc);
      return false;
    }
    return true;
  }

  inline void rcl_soft_check(rcl_ret_t rc, const char* where)
  {
    if (rc != RCL_RET_OK && g_micro_ros_node_instance)
    {
      g_micro_ros_node_instance->log_debugf("RCSOFTCHECK failed at %s rc=%d", where, (int) rc);
    }
  }

} // namespace

// ctor
MicroRosNode::MicroRosNode(MotionController& motion_controller, ImuBno055& imu, SonarHcsr04& sonar,
                           HwMotorDriver& motor_driver)
    : _motion_controller{motion_controller}, _imu{imu}, _sonar{sonar}, _motor_driver{motor_driver}
{
  g_micro_ros_node_instance = this;
}

bool MicroRosNode::init() {
  _init_transport();

  _init_core();

  _init_messaging();

  _sync_time(INIT_TIMEOUT_MS);

  _create_entities();

  return true;
}

void MicroRosNode::spin_executor(uint32_t budget_us)
{
  rcl_ret_t rc = rclc_executor_spin_some(&_executor, static_cast<uint64_t>(budget_us) * 1000ULL);
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    log_debugf("executor spin error: %d\n", (int) rc);
  }

  // periodic resync
  if ((millis() - _last_resync_ms) >= RESYNC_PERIOD_MS)
  {
    _sync_time(RESYNC_TIMEOUT_MS);
    log_debugf("resynced");
  }
}

void MicroRosNode::publish_encoder_ticks()
{
  WheelEncoderCounts encoder_counts{};
  if (!_motor_driver.read_encoder_counts(encoder_counts))
  {
    log_debugf("failed to read encoder counts");
    return;
  }

  _set_stamp_from_local_us(_encoder_msg.stamp, encoder_counts.timestamp_us);

  // FR RR FL RL
  _encoder_msg.front_right = encoder_counts.data[0];
  _encoder_msg.rear_right  = encoder_counts.data[1];
  _encoder_msg.front_left  = encoder_counts.data[2];
  _encoder_msg.rear_left   = encoder_counts.data[3];

  rcl_soft_check(rcl_publish(&_encoder_pub, &_encoder_msg, nullptr), "encoder publish");
}

void MicroRosNode::publish_imu()
{
  const ImuSample& reading = _imu.latest();

  _set_stamp_from_local_us(_imu_msg.header.stamp, reading.t_us);

  _imu_msg.orientation.x = reading.qx;
  _imu_msg.orientation.y = reading.qy;
  _imu_msg.orientation.z = reading.qz;
  _imu_msg.orientation.w = reading.qw;

  _imu_msg.angular_velocity.x = reading.wx;
  _imu_msg.angular_velocity.y = reading.wy;
  _imu_msg.angular_velocity.z = reading.wz;

  _imu_msg.linear_acceleration.x = reading.ax;
  _imu_msg.linear_acceleration.y = reading.ay;
  _imu_msg.linear_acceleration.z = reading.az;

  rcl_soft_check(rcl_publish(&_imu_pub, &_imu_msg, nullptr), "imu publish");
}

void MicroRosNode::publish_sonar()
{
  auto reading = _sonar.latest();
  if (!reading.valid) {
    return;
  }

  _set_stamp_from_local_us(_sonar_msg.header.stamp, reading.stamp_us);

  _sonar_msg.range = reading.range_m;

  rcl_soft_check(rcl_publish(&_sonar_pub, &_sonar_msg, nullptr), "sonar publish");
}

void MicroRosNode::publish_battery()
{
  float voltage = _motor_driver.read_battery_voltage();

  uint64_t now_us = micros();
  _set_stamp_from_local_us(_battery_msg.header.stamp, now_us);

  _battery_msg.voltage = voltage;
  _battery_msg.present = true;

  rcl_soft_check(rcl_publish(&_battery_pub, &_battery_msg, nullptr), "battery publish");
}

void MicroRosNode::cmd_vel_callback(const void* msg_in)
{
  if (!g_micro_ros_node_instance) {
    return;
  }

  const geometry_msgs__msg__Twist* msg =
      static_cast<const geometry_msgs__msg__Twist*>(msg_in);

  g_micro_ros_node_instance->_handle_cmd_vel(*msg);
}

void MicroRosNode::log_debugf(const char* fmt, ...)
{
  if (!_debug_ready)
  {
    return;
  }

  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(_debug_buf, sizeof(_debug_buf), fmt, args);
  va_end(args);

  if (n < 0)
  {
    return;
  }
  _debug_buf[sizeof(_debug_buf) - 1] = '\0';

  rosidl_runtime_c__String__assign(&_debug_msg.data, _debug_buf);

  (void) rcl_publish(&_debug_pub, &_debug_msg, nullptr);
}

void MicroRosNode::_init_transport()
{
  set_microros_serial_transports(Serial);
}

bool MicroRosNode::_init_core()
{
  // allocator
  _allocator = rcl_get_default_allocator();

  // build init options and set domain id
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (!rcl_check(rcl_init_options_init(&init_options, _allocator), "init_options init"))
  {
    return false;
  }
  if (!rcl_check(rcl_init_options_set_domain_id(&init_options, micro_ros_config::DOMAIN_ID),
                 "set domain id"))
  {
    return false;
  }

  if (!rcl_check(rclc_support_init_with_options(&_support, 0, nullptr, &init_options, &_allocator),
                 "support init with options"))
  {
    return false;
  }

  rcl_init_options_fini(&init_options);

  // node
  if (!rcl_check(rclc_node_init_default(&_node, NODE_NAME, NODE_NAMESPACE, &_support), "node init"))
  {
    return false;
  }

  // executor
  if (!rcl_check(rclc_executor_init(&_executor, &_support.context, 1, &_allocator),
                 "executor init"))
  {
    return false;
  }

  return true;
}

void MicroRosNode::_init_messaging()
{
  sensor_msgs__msg__Imu__init(&_imu_msg);
  sensor_msgs__msg__Range__init(&_sonar_msg);
  sensor_msgs__msg__BatteryState__init(&_battery_msg);

  rosidl_runtime_c__String__assign(&_imu_msg.header.frame_id, "imu_link");
  rosidl_runtime_c__String__assign(&_sonar_msg.header.frame_id, "sonar_link");
  rosidl_runtime_c__String__assign(&_battery_msg.header.frame_id,
                                   "base_link");

  _sonar_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  _sonar_msg.field_of_view  = _sonar.fov();
  _sonar_msg.min_range      = _sonar.min_range_m();
  _sonar_msg.max_range      = _sonar.max_range_m();
}

void MicroRosNode::_sync_time(int timeout_ms)
{
  rmw_uros_sync_session(timeout_ms);
  _time_synced = rmw_uros_epoch_synchronized();

  if (_time_synced)
  {
    const uint64_t agent_ns = static_cast<uint64_t>(rmw_uros_epoch_nanos());
    const uint64_t local_ns = static_cast<uint64_t>(micros()) * 1000ULL;
    _agent_offset_ns        = static_cast<int64_t>(agent_ns) - static_cast<int64_t>(local_ns);
    _last_resync_ms         = millis();

    log_debugf("[micro-ROS] time sync ok, offset = %lld ns\n",
               static_cast<long long>(_agent_offset_ns));
  }
  else
  {
    log_debugf("[micro-ROS] time sync FAILED\n");
  }
}

void MicroRosNode::_handle_cmd_vel(const geometry_msgs__msg__Twist& msg)
{
  log_debugf("in handle_ vel_callback");
  CmdVel cmd_vel{(float) msg.linear.x, (float) msg.linear.y, (float) msg.angular.z};
  _motion_controller.set_cmd_vel(cmd_vel, micros());
}

void MicroRosNode::_create_entities()
{
  // debug publisher
  rcl_soft_check(rclc_publisher_init_default(&_debug_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                             DEBUG_TOPIC),
                 "debug publish create");

  std_msgs__msg__String__init(&_debug_msg);
  _debug_ready = true;

  // encoder counts publisher
  rcl_soft_check(rclc_publisher_init_default(&_encoder_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(omniseer_msgs, msg,
                                                                         WheelEncoderCounts),
                                             ENCODER_TOPIC),
                 "encoder publish create");

  // IMU publisher
  rcl_soft_check(rclc_publisher_init_default(&_imu_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                             IMU_TOPIC),
                 "imu publish create");

  // sonar range publisher
  rcl_soft_check(rclc_publisher_init_default(&_sonar_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                             SONAR_TOPIC),
                 "sonar publish create");

  // battery publisher
  rcl_soft_check(rclc_publisher_init_default(&_battery_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,
                                                                         BatteryState),
                                             BATTERY_TOPIC),
                 "battery publish create");

  // cmd_vel subscriber
  rcl_soft_check(rclc_subscription_init_default(&_cmd_vel_sub, &_node,
                                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg,
                                                                            Twist),
                                                CMD_VEL_TOPIC),
                 "cmd vel subscriber create");

  rcl_soft_check(rclc_executor_add_subscription(&_executor, &_cmd_vel_sub, &_cmd_vel_msg,
                                                &cmd_vel_callback, ON_NEW_DATA),
                 "cmd vel subscriber add");
}

void MicroRosNode::_set_stamp_from_local_us(builtin_interfaces__msg__Time& t, uint64_t local_us)
{
  uint64_t local_ns = local_us * 1000ULL;

  uint64_t agent_ns;

  int64_t tmp = static_cast<int64_t>(local_ns) + _agent_offset_ns;
  agent_ns    = static_cast<uint64_t>(tmp);

  t.sec     = static_cast<int32_t>(agent_ns / 1000000000ULL);
  t.nanosec = static_cast<uint32_t>(agent_ns % 1000000000ULL);
}
