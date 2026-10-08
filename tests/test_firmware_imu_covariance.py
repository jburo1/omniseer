from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MICRO_ROS_NODE = REPO_ROOT / "firmware/src/micro_ros_node.cpp"


def test_imu_gyro_z_covariance_is_initialized_with_a_positive_variance() -> None:
    source = MICRO_ROS_NODE.read_text(encoding="utf-8")

    assert "constexpr double IMU_GYRO_Z_VARIANCE_RAD2_PER_S2 = 1e-4;" in source
    assert (
        "_imu_msg.angular_velocity_covariance[8] = IMU_GYRO_Z_VARIANCE_RAD2_PER_S2;"
        in source
    )


def test_imu_gyro_z_covariance_is_set_during_message_initialization() -> None:
    source = MICRO_ROS_NODE.read_text(encoding="utf-8")
    init = source.split("bool MicroRosNode::init()", maxsplit=1)[1].split(
        "bool MicroRosNode::is_ready", maxsplit=1
    )[0]
    init_messaging = source.split("void MicroRosNode::_init_messaging()", maxsplit=1)[1].split(
        "void MicroRosNode::_sync_time", maxsplit=1
    )[0]

    # init() is also the reconnect path and recreates the message after teardown.
    assert "_fini_messaging();" in init
    assert "_init_messaging();" in init
    assert "sensor_msgs__msg__Imu__init(&_imu_msg);" in init_messaging
    assert "_imu_msg.angular_velocity_covariance[8] = IMU_GYRO_Z_VARIANCE_RAD2_PER_S2;" in init_messaging
