cmake_minimum_required(VERSION 3.20)
project(omniseer_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(omniseer_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)

add_library(omniseer_autonomy_core STATIC
  src/in_place_scan_controller.cpp
  src/target_centering_controller.cpp
)
target_compile_features(omniseer_autonomy_core PUBLIC cxx_std_17)
target_include_directories(omniseer_autonomy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(target_centering_node
  src/target_centering_node.cpp
)
target_compile_features(target_centering_node PUBLIC cxx_std_17)
target_link_libraries(target_centering_node PRIVATE
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${omniseer_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${yolo_msgs_TARGETS}
  rclcpp::rclcpp
  omniseer_autonomy_core
)

add_executable(in_place_scan_node
  src/in_place_scan_node.cpp
)
target_compile_features(in_place_scan_node PUBLIC cxx_std_17)
target_link_libraries(in_place_scan_node PRIVATE
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  omniseer_autonomy_core
)

install(TARGETS
  omniseer_autonomy_core
  target_centering_node
  in_place_scan_node
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
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_target_centering_controller
    test/test_target_centering_controller.cpp
  )
  target_compile_features(test_target_centering_controller PUBLIC cxx_std_17)
  target_link_libraries(test_target_centering_controller
    omniseer_autonomy_core
  )
  target_include_directories(test_target_centering_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_target_centering_node
    test/test_target_centering_node.cpp
  )
  target_compile_features(test_target_centering_node PUBLIC cxx_std_17)
  target_link_libraries(test_target_centering_node
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${omniseer_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${yolo_msgs_TARGETS}
    rclcpp::rclcpp
    omniseer_autonomy_core
  )
  target_include_directories(test_target_centering_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_in_place_scan_controller
    test/test_in_place_scan_controller.cpp
  )
  target_compile_features(test_in_place_scan_controller PUBLIC cxx_std_17)
  target_link_libraries(test_in_place_scan_controller
    omniseer_autonomy_core
  )
  target_include_directories(test_in_place_scan_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
