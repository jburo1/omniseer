cmake_minimum_required(VERSION 3.8)
project(omniseer_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(xacro REQUIRED)

set(XACRO_IN        ${CMAKE_CURRENT_SOURCE_DIR}/urdf/omniseer.xacro)
set(URDF_OUT        ${CMAKE_CURRENT_BINARY_DIR}/omniseer.urdf)

add_custom_command(
  OUTPUT  ${URDF_OUT}
  COMMAND xacro ${XACRO_IN} -o ${URDF_OUT}
  DEPENDS ${XACRO_IN}
  COMMENT "Generating URDF from xacro")
  
install(DIRECTORY urdf meshes
        DESTINATION share/${PROJECT_NAME})

#  – Generated artefacts go in the same urdf folder so URI logic is simple
install(FILES ${URDF_OUT} ${EXTRA_MODEL_FILES}
        DESTINATION share/${PROJECT_NAME}/urdf)


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