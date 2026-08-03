cmake_minimum_required(VERSION 3.20)
project(robot_io_adapters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(omniseer_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(robot_io_adapters_math STATIC
  src/odometry_math.cpp
)
target_compile_features(robot_io_adapters_math PUBLIC cxx_std_17)
target_include_directories(robot_io_adapters_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(robot_io_adapters_node STATIC
  src/encoder_counts_to_odometry_node.cpp
)
target_compile_features(robot_io_adapters_node PUBLIC cxx_std_17)
target_include_directories(robot_io_adapters_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_io_adapters_node PUBLIC
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${omniseer_msgs_TARGETS}
  rclcpp::rclcpp
  robot_io_adapters_math
)

add_library(robot_io_adapters_scan_range STATIC
  src/scan_range_conversion.cpp
)
target_compile_features(robot_io_adapters_scan_range PUBLIC cxx_std_17)
target_include_directories(robot_io_adapters_scan_range PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_io_adapters_scan_range PUBLIC
  ${sensor_msgs_TARGETS}
)

add_library(robot_io_adapters_scan_to_range_node STATIC
  src/scan_to_range_node.cpp
)
target_compile_features(robot_io_adapters_scan_to_range_node PUBLIC cxx_std_17)
target_include_directories(robot_io_adapters_scan_to_range_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_io_adapters_scan_to_range_node PUBLIC
  rclcpp::rclcpp
  robot_io_adapters_scan_range
  ${sensor_msgs_TARGETS}
)

add_executable(encoder_counts_to_odometry
  src/encoder_counts_to_odometry_main.cpp
)
target_compile_features(encoder_counts_to_odometry PUBLIC cxx_std_17)
target_link_libraries(encoder_counts_to_odometry PRIVATE
  robot_io_adapters_node
)

add_executable(scan_to_range
  src/scan_to_range_main.cpp
)
target_compile_features(scan_to_range PUBLIC cxx_std_17)
target_link_libraries(scan_to_range PRIVATE
  robot_io_adapters_scan_to_range_node
)

install(TARGETS
  robot_io_adapters_math
  robot_io_adapters_node
  robot_io_adapters_scan_range
  robot_io_adapters_scan_to_range_node
  encoder_counts_to_odometry
  scan_to_range
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_odometry_math
    test/test_odometry_math.cpp
  )
  target_compile_features(test_odometry_math PUBLIC cxx_std_17)
  target_link_libraries(test_odometry_math
    robot_io_adapters_math
  )
  target_include_directories(test_odometry_math PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_scan_range_conversion
    test/test_scan_range_conversion.cpp
  )
  target_compile_features(test_scan_range_conversion PUBLIC cxx_std_17)
  target_link_libraries(test_scan_range_conversion
    robot_io_adapters_scan_range
  )
  target_include_directories(test_scan_range_conversion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

endif()

ament_package()
