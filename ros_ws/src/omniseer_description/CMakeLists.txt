cmake_minimum_required(VERSION 3.8)
project(omniseer_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
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

# --- Core (ROS-agnostic) ---
add_library(omniseer_frontier_core STATIC
  src/frontier.cpp
  src/grid_io.cpp
)

target_include_directories(omniseer_frontier_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>)

target_compile_features(omniseer_frontier_core PUBLIC cxx_std_17)

# --- ROS BT plugin ---
add_library(omniseer_bt_nodes SHARED
  src/select_frontier_from_costmap.cpp
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

target_link_libraries(omniseer_bt_nodes
  PRIVATE omniseer_frontier_core
)

pluginlib_export_plugin_description_file(behaviortree_cpp plugin_description.xml)

# --- Install ---
install(DIRECTORY urdf meshes rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS omniseer_bt_nodes omniseer_frontier_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(OMNISEER_ARTIFACTS_DIR "${CMAKE_CURRENT_BINARY_DIR}/artifacts")
  set(TEST_INC "${CMAKE_CURRENT_SOURCE_DIR}/test/include")

  ament_add_gtest(test_grid_io test/test_grid_io.cpp)
  target_link_libraries(test_grid_io omniseer_frontier_core)
  target_include_directories(test_grid_io PRIVATE ${TEST_INC})
  target_compile_definitions(test_grid_io PRIVATE
    TEST_DIR="${CMAKE_SOURCE_DIR}"
  )

  ament_add_gtest(test_frontier_mask test/test_frontier_mask.cpp)
  target_link_libraries(test_frontier_mask omniseer_frontier_core)
  target_include_directories(test_frontier_mask PRIVATE ${TEST_INC})
  target_compile_definitions(test_frontier_mask PRIVATE
    TEST_DIR="${CMAKE_SOURCE_DIR}"
    OMNISEER_ARTIFACTS_DIR="${OMNISEER_ARTIFACTS_DIR}"
  )

  ament_add_gtest(test_components test/test_components.cpp)
  target_link_libraries(test_components omniseer_frontier_core)
  target_include_directories(test_components PRIVATE ${TEST_INC})
  target_compile_definitions(test_components PRIVATE
    TEST_DIR="${CMAKE_SOURCE_DIR}"
    OMNISEER_ARTIFACTS_DIR="${OMNISEER_ARTIFACTS_DIR}"
  )

  ament_add_gtest(test_goals_select test/test_goals_select.cpp)
  target_link_libraries(test_goals_select omniseer_frontier_core)
  target_include_directories(test_goals_select PRIVATE ${TEST_INC})
  target_compile_definitions(test_goals_select PRIVATE
    TEST_DIR="${CMAKE_SOURCE_DIR}"
    OMNISEER_ARTIFACTS_DIR="${OMNISEER_ARTIFACTS_DIR}"
  )

  # more here
endif()

ament_package()
