cmake_minimum_required(VERSION 3.20)
project(robot_diag_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(omniseer_msgs REQUIRED)

set(robot_gateway_proto_root "${CMAKE_CURRENT_SOURCE_DIR}/../robot_diag_control")
set(robot_gateway_proto "${robot_gateway_proto_root}/robot_diag_control/api/robot_gateway.proto")
set(robot_gateway_codegen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(robot_gateway_pb_cc "${robot_gateway_codegen_dir}/robot_diag_control/api/robot_gateway.pb.cc")
set(robot_gateway_pb_h "${robot_gateway_codegen_dir}/robot_diag_control/api/robot_gateway.pb.h")
set(robot_gateway_grpc_pb_cc "${robot_gateway_codegen_dir}/robot_diag_control/api/robot_gateway.grpc.pb.cc")
set(robot_gateway_grpc_pb_h "${robot_gateway_codegen_dir}/robot_diag_control/api/robot_gateway.grpc.pb.h")

if(NOT EXISTS "${robot_gateway_proto}")
  message(FATAL_ERROR "Expected shared gateway proto at ${robot_gateway_proto}")
endif()

add_custom_command(
  OUTPUT
    "${robot_gateway_pb_cc}"
    "${robot_gateway_pb_h}"
    "${robot_gateway_grpc_pb_cc}"
    "${robot_gateway_grpc_pb_h}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${robot_gateway_codegen_dir}"
  COMMAND protobuf::protoc
  ARGS
    --proto_path=${robot_gateway_proto_root}
    --cpp_out=${robot_gateway_codegen_dir}
    --grpc_out=${robot_gateway_codegen_dir}
    --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    ${robot_gateway_proto}
  DEPENDS
    "${robot_gateway_proto}"
  VERBATIM
)

add_library(robot_diag_control_cpp_proto STATIC
  "${robot_gateway_pb_cc}"
  "${robot_gateway_grpc_pb_cc}"
)

target_compile_features(robot_diag_control_cpp_proto PUBLIC cxx_std_17)
target_include_directories(robot_diag_control_cpp_proto PUBLIC
  $<BUILD_INTERFACE:${robot_gateway_codegen_dir}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_diag_control_cpp_proto PUBLIC
  protobuf::libprotobuf
  gRPC::grpc++
)

add_library(robot_diag_control_cpp_core STATIC
  src/gateway_state.cpp
  src/preview_command_factory.cpp
  src/preview_process_manager.cpp
)

target_compile_features(robot_diag_control_cpp_core PUBLIC cxx_std_17)
target_include_directories(robot_diag_control_cpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_diag_control_cpp_core PUBLIC
  ${nav_msgs_TARGETS}
  ${omniseer_msgs_TARGETS}
)

add_library(robot_diag_control_cpp_grpc STATIC
  src/grpc_server.cpp
  src/robot_gateway_service.cpp
)

target_compile_features(robot_diag_control_cpp_grpc PUBLIC cxx_std_17)
target_include_directories(robot_diag_control_cpp_grpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_diag_control_cpp_grpc PUBLIC
  robot_diag_control_cpp_core
  robot_diag_control_cpp_proto
)

add_executable(robot_diag_control_cpp_node
  src/robot_diag_control_cpp_node.cpp
)

target_compile_features(robot_diag_control_cpp_node PUBLIC cxx_std_17)
target_link_libraries(robot_diag_control_cpp_node PRIVATE
  ${omniseer_msgs_TARGETS}
  rclcpp::rclcpp
  robot_diag_control_cpp_grpc
  robot_diag_control_cpp_core
)

install(TARGETS
  robot_diag_control_cpp_proto
  robot_diag_control_cpp_core
  robot_diag_control_cpp_grpc
  robot_diag_control_cpp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  "${robot_gateway_pb_h}"
  "${robot_gateway_grpc_pb_h}"
  DESTINATION include/robot_diag_control/api
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

  ament_add_gtest(test_gateway_state
    test/test_gateway_state.cpp
  )
  target_compile_features(test_gateway_state PUBLIC cxx_std_17)
  target_link_libraries(test_gateway_state
    robot_diag_control_cpp_proto
    ${omniseer_msgs_TARGETS}
    robot_diag_control_cpp_core
  )
  target_include_directories(test_gateway_state PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_preview_process_manager
    test/test_preview_process_manager.cpp
  )
  target_compile_features(test_preview_process_manager PUBLIC cxx_std_17)
  target_link_libraries(test_preview_process_manager
    robot_diag_control_cpp_core
  )
  target_include_directories(test_preview_process_manager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_preview_command_factory
    test/test_preview_command_factory.cpp
  )
  target_compile_features(test_preview_command_factory PUBLIC cxx_std_17)
  target_link_libraries(test_preview_command_factory
    robot_diag_control_cpp_core
  )
  target_include_directories(test_preview_command_factory PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_grpc_server
    test/test_grpc_server.cpp
  )
  target_compile_features(test_grpc_server PUBLIC cxx_std_17)
  target_link_libraries(test_grpc_server
    robot_diag_control_cpp_grpc
  )
  target_include_directories(test_grpc_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
