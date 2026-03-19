cmake_minimum_required(VERSION 3.20)
project(omniseer_vision_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omniseer_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)

pkg_check_modules(RGA REQUIRED librga)
find_path(RKNN_INCLUDE_DIR NAMES rknn_api.h REQUIRED)
find_library(RKNNRT_LIBRARY NAMES rknnrt REQUIRED)

set(VISION_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../vision)

add_library(omniseer_bridge_runtime STATIC
  src/vision_bridge_runtime.cpp
  ${VISION_ROOT}/src/class_list.cpp
  ${VISION_ROOT}/src/consumer_pipeline.cpp
  ${VISION_ROOT}/src/dma_heap_alloc.cpp
  ${VISION_ROOT}/src/image_buffer_pool.cpp
  ${VISION_ROOT}/src/pipeline.cpp
  ${VISION_ROOT}/src/producer_pipeline.cpp
  ${VISION_ROOT}/src/rga_preprocess.cpp
  ${VISION_ROOT}/src/rknn_runner.cpp
  ${VISION_ROOT}/src/spsc_ring.cpp
  ${VISION_ROOT}/src/time_utils.cpp
  ${VISION_ROOT}/src/v4l2_capture.cpp
  ${VISION_ROOT}/src/yolo_world_postprocess.cpp
  ${VISION_ROOT}/src/yolo_world_text_embeddings.cpp
)

target_compile_features(omniseer_bridge_runtime PRIVATE cxx_std_17)
target_include_directories(omniseer_bridge_runtime PRIVATE
  include
  ${VISION_ROOT}/include
  ${VISION_ROOT}/src
  ${RGA_INCLUDE_DIRS}
  ${RKNN_INCLUDE_DIR}
)
target_link_libraries(omniseer_bridge_runtime PRIVATE
  Threads::Threads
  ${RGA_LIBRARIES}
  ${RKNNRT_LIBRARY}
)
target_compile_options(omniseer_bridge_runtime PRIVATE ${RGA_CFLAGS_OTHER})

add_executable(vision_bridge_node
  src/vision_bridge_node.cpp
)

target_compile_features(vision_bridge_node PRIVATE cxx_std_17)
target_include_directories(vision_bridge_node PRIVATE
  include
  ${VISION_ROOT}/include
)
target_link_libraries(vision_bridge_node PRIVATE
  ${builtin_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${omniseer_msgs_TARGETS}
  ${yolo_msgs_TARGETS}
  rclcpp::rclcpp
  omniseer_bridge_runtime
)

install(TARGETS vision_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
