cmake_minimum_required(VERSION 3.8)
project(omniseer_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  # xacro regeneration test
  ament_add_test(check_xacro
    COMMAND bash -c "
      xacro ${CMAKE_SOURCE_DIR}/urdf/xacro/exported_robot.urdf.xacro \
        > ${CMAKE_BINARY_DIR}/rebuilt.urdf &&
      diff -u --ignore-all-space ${CMAKE_SOURCE_DIR}/urdf/exported_robot.urdf \
              ${CMAKE_BINARY_DIR}/rebuilt.urdf
    "
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})


  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
