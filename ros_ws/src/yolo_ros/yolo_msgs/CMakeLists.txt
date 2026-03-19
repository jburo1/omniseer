cmake_minimum_required(VERSION 3.8)
project(yolo_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Skip the deprecated Rust generator. yolo_msgs is consumed from C++ and Python
# only, and the Rust template expansion is currently unstable in this setup.
if(DEFINED AMENT_EXTENSIONS_rosidl_generate_idl_interfaces)
  set(_omniseer_rosidl_extensions "")
  foreach(_extension ${AMENT_EXTENSIONS_rosidl_generate_idl_interfaces})
    if(NOT _extension MATCHES "^rosidl_generator_rs:")
      list(APPEND _omniseer_rosidl_extensions "${_extension}")
    endif()
  endforeach()
  set(AMENT_EXTENSIONS_rosidl_generate_idl_interfaces "${_omniseer_rosidl_extensions}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Vector2.msg"
  "msg/Pose2D.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/Mask.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint2DArray.msg"
  "msg/KeyPoint3D.msg"
  "msg/KeyPoint3DArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "srv/SetClasses.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
