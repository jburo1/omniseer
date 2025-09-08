cmake_minimum_required(VERSION 3.8)
project(omniseer_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(omniseer_bt_nodes SHARED
  src/select_frontier_from_costmap.cpp
  src/frontier.cpp
  src/grid_io.cpp
)

target_include_directories(omniseer_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>)

target_compile_features(omniseer_bt_nodes PUBLIC cxx_std_17)

target_link_libraries(omniseer_bt_nodes
  PUBLIC
    ${geometry_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    behaviortree_cpp::behaviortree_cpp
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros)

pluginlib_export_plugin_description_file(behaviortree_cpp plugin_description.xml)

install(DIRECTORY urdf meshes rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS omniseer_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  # grid io test
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_io test/test_grid_io.cpp)
  target_link_libraries(test_grid_io omniseer_bt_nodes)
  target_compile_definitions(test_grid_io PRIVATE TEST_DIR="${CMAKE_SOURCE_DIR}")

  # # xacro regeneration test
  # ament_add_test(check_xacro
  #   COMMAND bash -c "
  #     xacro ${CMAKE_SOURCE_DIR}/urdf/xacro/exported_robot.urdf.xacro \
  #       > ${CMAKE_BINARY_DIR}/rebuilt.urdf &&
  #     diff -u --ignore-all-space ${CMAKE_SOURCE_DIR}/urdf/exported_robot.urdf \
  #             ${CMAKE_BINARY_DIR}/rebuilt.urdf
  #   "
  #   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
