cmake_minimum_required(VERSION 3.20)
project(omniseer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Kilted pulls in the deprecated Rust generator by default via
# rosidl_default_generators. This workspace doesn't consume Rust interfaces,
# and the generator currently breaks on this environment's Python toolchain.
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
  "msg/VisionPerfSummary.msg"
  "msg/WheelEncoderCounts.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
