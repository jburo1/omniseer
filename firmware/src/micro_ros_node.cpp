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
  _reset_runtime_state();
}

bool MicroRosNode::init() {
  if (_ready)
  {
    return true;
  }

  _fini_entities();
  _fini_messaging();
  _fini_core();
  _reset_runtime_state();

  _serial_logf("init attempt");
  _init_transport();

  if (!_ping_agent(INIT_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
  {
    _reset_runtime_state();
    return false;
  }

  if (!_init_core())
  {
    _fini_core();
    _reset_runtime_state();
    return false;
  }

  _init_messaging();

  _sync_time(INIT_TIMEOUT_MS);

  if (!_create_entities())
  {
    _fini_entities();
    _fini_messaging();
    _fini_core();
    _reset_runtime_state();
    return false;
  }

  _ready = true;
  _last_agent_ping_ms = millis();
  log_debugf("init ok");
  return true;
}

bool MicroRosNode::is_ready() const
{
  return _ready;
}

void MicroRosNode::spin_executor(uint32_t budget_us)
{
  if (!_ready)
  {
    return;
  }

  if (!Serial.dtr())
  {
    _handle_disconnect("serial dtr dropped", 0);
    return;
  }

  rcl_ret_t rc = rclc_executor_spin_some(&_executor, static_cast<uint64_t>(budget_us) * 1000ULL);
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT)
  {
    _handle_disconnect("executor spin", static_cast<int>(rc));
    return;
  }

  // periodic resync
  const uint32_t now_ms = millis();
  if ((now_ms - _last_resync_ms) >= RESYNC_PERIOD_MS)
  {
    _sync_time(RESYNC_TIMEOUT_MS);
    log_debugf("resynced");
  }
}

void MicroRosNode::publish_encoder_ticks()
{
  if (!_ready)
  {
    return;
  }

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
  if (!_ready)
  {
    return;
  }

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
  if (!_ready)
  {
    return;
  }

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
  if (!_ready)
  {
    return;
  }

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

  const geometry_msgs__msg__TwistStamped* msg =
      static_cast<const geometry_msgs__msg__TwistStamped*>(msg_in);

  g_micro_ros_node_instance->_handle_cmd_vel(*msg);
}

void MicroRosNode::log_debugf(const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(_debug_buf, sizeof(_debug_buf), fmt, args);
  va_end(args);

  if (n < 0)
  {
    return;
  }
  _debug_buf[sizeof(_debug_buf) - 1] = '\0';

  if (!_debug_ready)
  {
    return;
  }

  rosidl_runtime_c__String__assign(&_debug_msg.data, _debug_buf);

  rcl_soft_check(rcl_publish(&_debug_pub, &_debug_msg, nullptr), "debug publish");
}

void MicroRosNode::_serial_logf(const char* fmt, ...)
{
  if (!_serial_logging_allowed)
  {
    return;
  }

  char    serial_buf[DEBUG_BUF_SIZE];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(serial_buf, sizeof(serial_buf), fmt, args);
  va_end(args);

  if (n < 0)
  {
    return;
  }

  serial_buf[sizeof(serial_buf) - 1] = '\0';
  Serial.print("[micro-ROS] ");
  Serial.println(serial_buf);
}

void MicroRosNode::_init_transport()
{
  set_microros_serial_transports(Serial);
  _transport_initialized = true;
  _serial_logging_allowed = false;
}

bool MicroRosNode::_init_core()
{
  // allocator
  _allocator = rcl_get_default_allocator();
  _support = {};
  _executor = {};
  _node = rcl_get_zero_initialized_node();
  _debug_pub = rcl_get_zero_initialized_publisher();
  _encoder_pub = rcl_get_zero_initialized_publisher();
  _imu_pub = rcl_get_zero_initialized_publisher();
  _sonar_pub = rcl_get_zero_initialized_publisher();
  _battery_pub = rcl_get_zero_initialized_publisher();
  _cmd_vel_sub = rcl_get_zero_initialized_subscription();

  // build init options and set domain id
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (!rcl_check(rcl_init_options_init(&init_options, _allocator), "init_options init"))
  {
    rcl_soft_check(rcl_init_options_fini(&init_options), "init options fini");
    return false;
  }
  if (!rcl_check(rcl_init_options_set_domain_id(&init_options, micro_ros_config::DOMAIN_ID),
                 "set domain id"))
  {
    rcl_soft_check(rcl_init_options_fini(&init_options), "init options fini");
    return false;
  }

  if (!rcl_check(rclc_support_init_with_options(&_support, 0, nullptr, &init_options, &_allocator),
                 "support init with options"))
  {
    rcl_soft_check(rcl_init_options_fini(&init_options), "init options fini");
    return false;
  }
  _support_ready = true;

  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
  if (!rmw_context)
  {
    return false;
  }

  if (rmw_uros_set_context_entity_creation_session_timeout(rmw_context,
                                                           INIT_TIMEOUT_MS) != RMW_RET_OK)
  {
    return false;
  }

  if (rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0) != RMW_RET_OK)
  {
    return false;
  }

  rcl_soft_check(rcl_init_options_fini(&init_options), "init options fini");

  // node
  if (!rcl_check(rclc_node_init_default(&_node, NODE_NAME, NODE_NAMESPACE, &_support), "node init"))
  {
    return false;
  }
  _node_ready = true;

  // executor
  if (!rcl_check(rclc_executor_init(&_executor, &_support.context, 1, &_allocator),
                 "executor init"))
  {
    return false;
  }
  _executor_ready = true;

  return true;
}

void MicroRosNode::_init_messaging()
{
  _encoder_msg = {};
  _imu_msg = {};
  _cmd_vel_msg = {};
  _sonar_msg = {};
  _battery_msg = {};
  _debug_msg = {};

  sensor_msgs__msg__Imu__init(&_imu_msg);
  geometry_msgs__msg__TwistStamped__init(&_cmd_vel_msg);
  sensor_msgs__msg__Range__init(&_sonar_msg);
  sensor_msgs__msg__BatteryState__init(&_battery_msg);
  std_msgs__msg__String__init(&_debug_msg);

  rosidl_runtime_c__String__assign(&_imu_msg.header.frame_id, "imu_link");
  rosidl_runtime_c__String__assign(&_sonar_msg.header.frame_id, "sonar_link");
  rosidl_runtime_c__String__assign(&_battery_msg.header.frame_id,
                                   "base_link");

  _sonar_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  _sonar_msg.field_of_view  = _sonar.fov();
  _sonar_msg.min_range      = _sonar.min_range_m();
  _sonar_msg.max_range      = _sonar.max_range_m();

  _messaging_ready = true;
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

void MicroRosNode::_handle_cmd_vel(const geometry_msgs__msg__TwistStamped& msg)
{
  log_debugf("in handle_ vel_callback");
  CmdVel cmd_vel{
      static_cast<float>(msg.twist.linear.x),
      static_cast<float>(msg.twist.linear.y),
      static_cast<float>(msg.twist.angular.z),
  };
  _motion_controller.set_cmd_vel(cmd_vel, micros());
}

bool MicroRosNode::_create_entities()
{
  // debug publisher
  if (!rcl_check(rclc_publisher_init_default(&_debug_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                             DEBUG_TOPIC),
                 "debug publish create"))
  {
    return false;
  }
  _debug_pub_ready = true;
  _debug_ready = true;

  // encoder counts publisher
  if (!rcl_check(rclc_publisher_init_default(&_encoder_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(omniseer_msgs, msg,
                                                                         WheelEncoderCounts),
                                             ENCODER_TOPIC),
                 "encoder publish create"))
  {
    return false;
  }
  _encoder_pub_ready = true;

  // IMU publisher
  if (!rcl_check(rclc_publisher_init_default(&_imu_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                             IMU_TOPIC),
                 "imu publish create"))
  {
    return false;
  }
  _imu_pub_ready = true;

  // sonar range publisher
  if (!rcl_check(rclc_publisher_init_default(&_sonar_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                             SONAR_TOPIC),
                 "sonar publish create"))
  {
    return false;
  }
  _sonar_pub_ready = true;

  // battery publisher
  if (!rcl_check(rclc_publisher_init_default(&_battery_pub, &_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,
                                                                         BatteryState),
                                             BATTERY_TOPIC),
                 "battery publish create"))
  {
    return false;
  }
  _battery_pub_ready = true;

  // cmd_vel subscriber
  if (!rcl_check(rclc_subscription_init_default(&_cmd_vel_sub, &_node,
                                                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg,
                                                                            TwistStamped),
                                                CMD_VEL_TOPIC),
                 "cmd vel subscriber create"))
  {
    return false;
  }
  _cmd_vel_sub_ready = true;

  if (!rcl_check(rclc_executor_add_subscription(&_executor, &_cmd_vel_sub, &_cmd_vel_msg,
                                                &cmd_vel_callback, ON_NEW_DATA),
                 "cmd vel subscriber add"))
  {
    return false;
  }

  return true;
}

bool MicroRosNode::_ping_agent(int timeout_ms, uint8_t attempts)
{
  return rmw_uros_ping_agent(timeout_ms, attempts) == RMW_RET_OK;
}

void MicroRosNode::_handle_disconnect(const char* where, int rc)
{
  (void) where;
  (void) rc;
  _stop_motion();
  _fini_entities();
  _fini_messaging();
  _fini_core();
  _reset_runtime_state();
}

void MicroRosNode::_fini_entities()
{
  if (_cmd_vel_sub_ready && _node_ready)
  {
    rcl_soft_check(rcl_subscription_fini(&_cmd_vel_sub, &_node), "cmd vel subscriber fini");
  }
  _cmd_vel_sub = rcl_get_zero_initialized_subscription();
  _cmd_vel_sub_ready = false;

  if (_battery_pub_ready && _node_ready)
  {
    rcl_soft_check(rcl_publisher_fini(&_battery_pub, &_node), "battery publish fini");
  }
  _battery_pub = rcl_get_zero_initialized_publisher();
  _battery_pub_ready = false;

  if (_sonar_pub_ready && _node_ready)
  {
    rcl_soft_check(rcl_publisher_fini(&_sonar_pub, &_node), "sonar publish fini");
  }
  _sonar_pub = rcl_get_zero_initialized_publisher();
  _sonar_pub_ready = false;

  if (_imu_pub_ready && _node_ready)
  {
    rcl_soft_check(rcl_publisher_fini(&_imu_pub, &_node), "imu publish fini");
  }
  _imu_pub = rcl_get_zero_initialized_publisher();
  _imu_pub_ready = false;

  if (_encoder_pub_ready && _node_ready)
  {
    rcl_soft_check(rcl_publisher_fini(&_encoder_pub, &_node), "encoder publish fini");
  }
  _encoder_pub = rcl_get_zero_initialized_publisher();
  _encoder_pub_ready = false;

  if (_debug_pub_ready && _node_ready)
  {
    rcl_soft_check(rcl_publisher_fini(&_debug_pub, &_node), "debug publish fini");
  }
  _debug_pub = rcl_get_zero_initialized_publisher();
  _debug_pub_ready = false;
  _debug_ready = false;
}

void MicroRosNode::_fini_messaging()
{
  if (!_messaging_ready)
  {
    return;
  }

  std_msgs__msg__String__fini(&_debug_msg);
  sensor_msgs__msg__BatteryState__fini(&_battery_msg);
  sensor_msgs__msg__Range__fini(&_sonar_msg);
  geometry_msgs__msg__TwistStamped__fini(&_cmd_vel_msg);
  sensor_msgs__msg__Imu__fini(&_imu_msg);

  _debug_msg = {};
  _battery_msg = {};
  _sonar_msg = {};
  _cmd_vel_msg = {};
  _imu_msg = {};
  _encoder_msg = {};
  _messaging_ready = false;
}

void MicroRosNode::_fini_core()
{
  if (_executor_ready)
  {
    rcl_soft_check(rclc_executor_fini(&_executor), "executor fini");
  }
  _executor = {};
  _executor_ready = false;

  if (_node_ready)
  {
    rcl_soft_check(rcl_node_fini(&_node), "node fini");
  }
  _node = rcl_get_zero_initialized_node();
  _node_ready = false;

  if (_support_ready)
  {
    rcl_soft_check(rclc_support_fini(&_support), "support fini");
  }
  _support = {};
  _support_ready = false;
}

void MicroRosNode::_reset_runtime_state()
{
  _allocator = rcl_get_default_allocator();
  _support = {};
  _node = rcl_get_zero_initialized_node();
  _executor = {};
  _debug_pub = rcl_get_zero_initialized_publisher();
  _encoder_pub = rcl_get_zero_initialized_publisher();
  _imu_pub = rcl_get_zero_initialized_publisher();
  _sonar_pub = rcl_get_zero_initialized_publisher();
  _battery_pub = rcl_get_zero_initialized_publisher();
  _cmd_vel_sub = rcl_get_zero_initialized_subscription();

  _encoder_msg = {};
  _imu_msg = {};
  _cmd_vel_msg = {};
  _sonar_msg = {};
  _battery_msg = {};
  _debug_msg = {};

  _time_synced = false;
  _agent_offset_ns = 0;
  _last_resync_ms = 0;
  _last_agent_ping_ms = 0;
  _debug_ready = false;
  _transport_initialized = false;
  _support_ready = false;
  _node_ready = false;
  _executor_ready = false;
  _messaging_ready = false;
  _debug_pub_ready = false;
  _encoder_pub_ready = false;
  _imu_pub_ready = false;
  _sonar_pub_ready = false;
  _battery_pub_ready = false;
  _cmd_vel_sub_ready = false;
  _ready = false;
}

void MicroRosNode::_stop_motion()
{
  _motion_controller.set_cmd_vel(CmdVel{0.0f, 0.0f, 0.0f}, micros());
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
