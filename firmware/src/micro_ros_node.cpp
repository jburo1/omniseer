// https://github.com/micro-ROS/micro-ROS-demos/blob/kilted/rclc/int32_publisher/main.c

#include "micro_ros_node.hpp"

// rcl error checkers (from gh)
#define RCCHECK_BOOL(fn)                                                            \
  {                                                                                 \
    rcl_ret_t temp_rc = (fn);                                                       \
    if (temp_rc != RCL_RET_OK) {                                                    \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);  \
      return false;                                                                 \
    }                                                                               \
  }

#define RCSOFTCHECK(fn)                                                             \
  {                                                                                 \
    rcl_ret_t temp_rc = (fn);                                                       \
    if (temp_rc != RCL_RET_OK) {                                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);\
    }                                                                               \
  }

static MicroRosNode* g_micro_ros_node_instance = nullptr;

// ctor
MicroRosNode::MicroRosNode(MotionController& motion_controller,
                 ImuBno055&       imu,
                 SonarHcsr04&     sonar,
                 HwMotorDriver&   motor_driver,
                 const MicroRosConfig& config) :
_motion_controller {motion_controller}, _imu{imu}, _sonar{sonar}, _motor_driver {motor_driver}, _config{config} {   g_micro_ros_node_instance = this;}

bool MicroRosNode::init() {
  _init_transport();

  // rlc core
  _allocator = rcl_get_default_allocator();
  RCCHECK_BOOL(rclc_support_init(&_support, 0, nullptr, &_allocator));

  // node
  RCCHECK_BOOL(rclc_node_init_default(&_node, _config.node_name, _config.node_namespace, &_support));

  // executor
  RCCHECK_BOOL(rclc_executor_init(&_executor, &_support.context, 1, &_allocator));

  // message init
  _sonar_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  _sonar_msg.field_of_view  = _sonar.fov();
  _sonar_msg.min_range      = _sonar.min_range_m();
  _sonar_msg.max_range      = _sonar.max_range_m();

  // pub/sub/callbacks
  _create_entities();

  return true;
}

void MicroRosNode::_create_entities()
{
  // Encoder counts publisher
  RCCHECK_BOOL(rclc_publisher_init_default(
      &_encoder_pub,
      &_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      _config.encoder_topic));

  // IMU publisher
  RCCHECK_BOOL(rclc_publisher_init_default(
      &_imu_pub,
      &_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      _config.imu_topic));

  // Sonar range publisher
  RCCHECK_BOOL(rclc_publisher_init_default(
      &_sonar_pub,
      &_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      _config.sonar_topic));

  // cmd_vel subscriber
  RCCHECK_BOOL(rclc_subscription_init_default(
      &_cmd_vel_sub,
      &_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      _config.cmd_vel_topic));

  // Hook cmd_vel into executor
  RCCHECK_BOOL(rclc_executor_add_subscription(
      &_executor,
      &_cmd_vel_sub,
      &_cmd_vel_msg,
      &cmd_vel_callback,
      ON_NEW_DATA));
}

void MicroRosNode::spin_executor(uint32_t budget_us)
{
  const uint64_t timeout_ns = static_cast<uint64_t>(budget_us) * 1000ULL;
  rcl_ret_t rc = rclc_executor_spin_some(&_executor, timeout_ns);
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    printf("executor spin error: %d\n", (int)rc);
  }
}

void MicroRosNode::_init_transport(){
  set_microros_serial_transports(Serial);
}

void MicroRosNode::_handle_cmd_vel(const geometry_msgs__msg__Twist& msg)
{
  CmdVel cmd_vel{msg.linear.x, msg.linear.y, msg.angular.z};
  _motion_controller.set_cmd_vel(cmd_vel, micros());
}

void MicroRosNode::publish_encoder_ticks()
{
  WheelEncoderCounts encoder_counts{};
  _motor_driver.read_encoder_counts(encoder_counts);

  const uint32_t sec  = encoder_counts.timestamp_us / 1000000U;
  const uint32_t usec = encoder_counts.timestamp_us % 1000000U;

  _encoder_msg.stamp.sec     = sec;
  _encoder_msg.stamp.nanosec = usec * 1000U;

  _encoder_msg.front_left  = enc.fl;
  _encoder_msg.front_right = enc.fr;
  _encoder_msg.rear_left   = enc.rl;
  _encoder_msg.rear_right  = enc.rr;

  RCSOFTCHECK(rcl_publish(&_encoder_pub, &_encoder_msg, nullptr));
}


void MicroRosNode::publish_imu()
{
  const ImuSample& reading = _imu.latest();

  const uint32_t sec  = reading.t_us / 1000000U;
  const uint32_t usec = reading.t_us % 1000000U;
  _imu_msg.header.stamp.sec     = sec;
  _imu_msg.header.stamp.nanosec = usec * 1000U;

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

  RCSOFTCHECK(rcl_publish(&_imu_pub, &_imu_msg, nullptr));
}

void MicroRosNode::publish_sonar()
{
  auto reading = _sonar.latest();
  if (!reading.valid) {
    return;
  }

  // time stamp
  uint32_t sec  = reading.stamp_us / 1000000U;
  uint32_t usec = reading.stamp_us % 1000000U;
  _sonar_msg.header.stamp.sec     = sec;
  _sonar_msg.header.stamp.nanosec = usec * 1000U;

  _sonar_msg.range = reading.range_m;

  RCSOFTCHECK(rcl_publish(&_sonar_pub, &_sonar_msg, nullptr));
}

static void cmd_vel_callback(const void* msg_in)
{
  if (!g_micro_ros_node_instance) {
    return;
  }

  const geometry_msgs__msg__Twist* msg =
      static_cast<const geometry_msgs__msg__Twist*>(msg_in);

  g_micro_ros_node_instance->_handle_cmd_vel(*msg);
}
