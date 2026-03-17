cmake_minimum_required(VERSION 3.16)
project(vision_harness CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)
find_path(RKNN_INCLUDE_DIR NAMES rknn_api.h)
find_library(RKNNRT_LIBRARY NAMES rknnrt)

option(VISION_BUILD_HARNESS "Build vision_harness app (requires librga)" ON)
option(VISION_BUILD_TESTS "Build vision tests" ON)

if(VISION_BUILD_HARNESS)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(RGA librga)
  pkg_check_modules(OPENCV4 opencv4)

  if(NOT RGA_FOUND)
    message(WARNING "librga not found; disabling vision_harness (set -DVISION_BUILD_HARNESS=OFF to silence)")
    set(VISION_BUILD_HARNESS OFF)
  elseif(NOT RKNN_INCLUDE_DIR OR NOT RKNNRT_LIBRARY)
    message(WARNING "RKNN runtime not found; disabling vision_harness (set -DVISION_BUILD_HARNESS=OFF to silence)")
    set(VISION_BUILD_HARNESS OFF)
  elseif(NOT OPENCV4_FOUND)
    message(WARNING "OpenCV not found; disabling vision_harness (set -DVISION_BUILD_HARNESS=OFF to silence)")
    set(VISION_BUILD_HARNESS OFF)
  endif()
endif()

set(VISION_HARNESS_SOURCES
  apps/vision_harness.cpp
  src/class_list.cpp
  src/consumer_pipeline.cpp
  src/dma_heap_alloc.cpp
  src/image_buffer_pool.cpp
  src/jsonl_telemetry.cpp
  src/pipeline.cpp
  src/producer_pipeline.cpp
  src/rga_preprocess.cpp
  src/rknn_runner.cpp
  src/spsc_ring.cpp
  src/time_utils.cpp
  src/v4l2_capture.cpp
  src/yolo_world_postprocess.cpp
  src/yolo_world_text_embeddings.cpp
)

if(VISION_BUILD_HARNESS)
  add_executable(vision_harness ${VISION_HARNESS_SOURCES})
  target_include_directories(vision_harness PRIVATE include ${RGA_INCLUDE_DIRS} ${RKNN_INCLUDE_DIR} ${OPENCV4_INCLUDE_DIRS})
  target_link_libraries(vision_harness PRIVATE ${RGA_LIBRARIES} ${RKNNRT_LIBRARY} ${OPENCV4_LIBRARIES})
  target_compile_options(vision_harness PRIVATE ${RGA_CFLAGS_OTHER} ${OPENCV4_CFLAGS_OTHER})
  target_compile_definitions(vision_harness PRIVATE VISION_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endif()

if(VISION_BUILD_HARNESS)
  set(VISION_LINT_SOURCES ${VISION_HARNESS_SOURCES})
else()
  set(VISION_LINT_SOURCES
    src/image_buffer_pool.cpp
    src/spsc_ring.cpp
  )
endif()

if(VISION_BUILD_TESTS)
  include(CTest)
  enable_testing()

  set(GTEST_SOURCE_DIR "" CACHE PATH "Path to a local googletest source tree (used if GTest isn't installed)")
  option(VISION_FETCH_GOOGLETEST "Fetch googletest via FetchContent when not found (requires network)" OFF)

  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    if(GTEST_SOURCE_DIR)
      add_subdirectory(${GTEST_SOURCE_DIR} ${CMAKE_BINARY_DIR}/_deps/googletest-build EXCLUDE_FROM_ALL)
    elseif(EXISTS "/usr/src/googletest/CMakeLists.txt")
      add_subdirectory(/usr/src/googletest ${CMAKE_BINARY_DIR}/_deps/googletest-build EXCLUDE_FROM_ALL)
    elseif(EXISTS "/usr/src/gtest/CMakeLists.txt")
      add_subdirectory(/usr/src/gtest ${CMAKE_BINARY_DIR}/_deps/googletest-build EXCLUDE_FROM_ALL)
    elseif(VISION_FETCH_GOOGLETEST)
      include(FetchContent)
      FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/release-1.14.0.zip
      )
      FetchContent_MakeAvailable(googletest)
    else()
      message(FATAL_ERROR "GTest not found. Install googletest (or provide -DGTEST_SOURCE_DIR=...), or set -DVISION_FETCH_GOOGLETEST=ON.")
    endif()

    if(NOT TARGET GTest::gtest AND TARGET gtest)
      add_library(GTest::gtest ALIAS gtest)
    endif()
    if(NOT TARGET GTest::gtest_main AND TARGET gtest_main)
      add_library(GTest::gtest_main ALIAS gtest_main)
    endif()
  endif()

  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(RGA_TEST librga)
  endif()

  add_executable(image_buffer_pool_test
    apps/image_buffer_pool_test.cpp
    src/dma_heap_alloc.cpp
    src/image_buffer_pool.cpp
    src/spsc_ring.cpp
  )
  target_include_directories(image_buffer_pool_test PRIVATE include)
  target_link_libraries(image_buffer_pool_test PRIVATE Threads::Threads GTest::gtest_main)

  add_test(NAME image_buffer_pool_test COMMAND image_buffer_pool_test)

  if(RKNN_INCLUDE_DIR AND RKNNRT_LIBRARY)
    add_executable(rknn_runner_test
      apps/rknn_runner_test.cpp
      src/dma_heap_alloc.cpp
      src/image_buffer_pool.cpp
      src/rknn_runner.cpp
      src/spsc_ring.cpp
    )
    target_include_directories(rknn_runner_test PRIVATE include ${RKNN_INCLUDE_DIR})
    target_link_libraries(rknn_runner_test PRIVATE Threads::Threads GTest::gtest_main ${RKNNRT_LIBRARY})
    target_compile_definitions(rknn_runner_test PRIVATE VISION_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

    add_test(NAME rknn_runner_test COMMAND rknn_runner_test)

    add_executable(consumer_pipeline_test
      apps/consumer_pipeline_test.cpp
      src/consumer_pipeline.cpp
      src/dma_heap_alloc.cpp
      src/image_buffer_pool.cpp
      src/rknn_runner.cpp
      src/spsc_ring.cpp
      src/yolo_world_postprocess.cpp
    )
    target_include_directories(consumer_pipeline_test PRIVATE include ${RKNN_INCLUDE_DIR})
    target_link_libraries(consumer_pipeline_test PRIVATE Threads::Threads GTest::gtest_main ${RKNNRT_LIBRARY})
    target_compile_definitions(consumer_pipeline_test PRIVATE VISION_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

    add_test(NAME consumer_pipeline_test COMMAND consumer_pipeline_test)

    add_executable(yolo_world_postprocess_test
      apps/yolo_world_postprocess_test.cpp
      src/yolo_world_postprocess.cpp
    )
    target_include_directories(yolo_world_postprocess_test PRIVATE include ${RKNN_INCLUDE_DIR})
    target_link_libraries(yolo_world_postprocess_test PRIVATE Threads::Threads GTest::gtest_main)

    add_test(NAME yolo_world_postprocess_test COMMAND yolo_world_postprocess_test)

    add_executable(yolo_world_text_embeddings_test
      apps/yolo_world_text_embeddings_test.cpp
      src/yolo_world_text_embeddings.cpp
    )
    target_include_directories(yolo_world_text_embeddings_test PRIVATE include ${RKNN_INCLUDE_DIR})
    target_link_libraries(yolo_world_text_embeddings_test PRIVATE Threads::Threads GTest::gtest_main ${RKNNRT_LIBRARY})
    target_compile_definitions(yolo_world_text_embeddings_test PRIVATE VISION_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

    add_test(NAME yolo_world_text_embeddings_test COMMAND yolo_world_text_embeddings_test)

    add_executable(jsonl_telemetry_test
      apps/jsonl_telemetry_test.cpp
      src/jsonl_telemetry.cpp
      src/spsc_ring.cpp
    )
    target_include_directories(jsonl_telemetry_test PRIVATE include ${RKNN_INCLUDE_DIR})
    target_link_libraries(jsonl_telemetry_test PRIVATE Threads::Threads GTest::gtest_main)

    add_test(NAME jsonl_telemetry_test COMMAND jsonl_telemetry_test)

    add_executable(rolling_telemetry_test
      apps/rolling_telemetry_test.cpp
    )
    target_include_directories(rolling_telemetry_test PRIVATE include ${RKNN_INCLUDE_DIR})
    target_link_libraries(rolling_telemetry_test PRIVATE Threads::Threads GTest::gtest_main)

    add_test(NAME rolling_telemetry_test COMMAND rolling_telemetry_test)
  else()
    message(STATUS "RKNN runtime not found; RKNN-dependent tests will not be built")
  endif()

  # Device-dependent V4L2 test (skips if device is unavailable).
  add_executable(v4l2_capture_test
    apps/v4l2_capture_test.cpp
    src/time_utils.cpp
    src/v4l2_capture.cpp
  )
  target_include_directories(v4l2_capture_test PRIVATE include)
  target_link_libraries(v4l2_capture_test PRIVATE Threads::Threads GTest::gtest_main)

  add_test(NAME v4l2_capture_test COMMAND v4l2_capture_test)

  # Hardware integration test for RGA preprocess (skips at runtime if /dev/rga is unavailable).
  if(RGA_TEST_FOUND)
    add_executable(rga_preprocess_test
      apps/rga_preprocess_test.cpp
      src/dma_heap_alloc.cpp
      src/rga_preprocess.cpp
      src/time_utils.cpp
      src/v4l2_capture.cpp
    )
    target_include_directories(rga_preprocess_test PRIVATE include ${RGA_TEST_INCLUDE_DIRS})
    target_link_libraries(rga_preprocess_test PRIVATE Threads::Threads GTest::gtest_main ${RGA_TEST_LIBRARIES})
    target_compile_options(rga_preprocess_test PRIVATE ${RGA_TEST_CFLAGS_OTHER})

    add_test(NAME rga_preprocess_test COMMAND rga_preprocess_test)

    add_executable(producer_pipeline_test
      apps/producer_pipeline_test.cpp
      src/dma_heap_alloc.cpp
      src/image_buffer_pool.cpp
      src/pipeline.cpp
      src/producer_pipeline.cpp
      src/rga_preprocess.cpp
      src/spsc_ring.cpp
      src/time_utils.cpp
      src/v4l2_capture.cpp
    )
    target_include_directories(producer_pipeline_test PRIVATE include ${RGA_TEST_INCLUDE_DIRS})
    target_link_libraries(producer_pipeline_test PRIVATE Threads::Threads GTest::gtest_main ${RGA_TEST_LIBRARIES})
    target_compile_options(producer_pipeline_test PRIVATE ${RGA_TEST_CFLAGS_OTHER})

    add_test(NAME producer_pipeline_test COMMAND producer_pipeline_test)

    if(RKNN_INCLUDE_DIR AND RKNNRT_LIBRARY)
      add_executable(vision_pipeline_integration_test
        apps/vision_pipeline_integration_test.cpp
        src/class_list.cpp
        src/consumer_pipeline.cpp
        src/dma_heap_alloc.cpp
        src/image_buffer_pool.cpp
        src/producer_pipeline.cpp
        src/rga_preprocess.cpp
        src/rknn_runner.cpp
        src/spsc_ring.cpp
        src/time_utils.cpp
        src/v4l2_capture.cpp
        src/yolo_world_postprocess.cpp
        src/yolo_world_text_embeddings.cpp
      )
      target_include_directories(vision_pipeline_integration_test PRIVATE include ${RGA_TEST_INCLUDE_DIRS} ${RKNN_INCLUDE_DIR})
      target_link_libraries(vision_pipeline_integration_test PRIVATE Threads::Threads GTest::gtest_main ${RGA_TEST_LIBRARIES} ${RKNNRT_LIBRARY})
      target_compile_options(vision_pipeline_integration_test PRIVATE ${RGA_TEST_CFLAGS_OTHER})
      target_compile_definitions(vision_pipeline_integration_test PRIVATE VISION_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

      add_test(NAME vision_pipeline_integration_test COMMAND vision_pipeline_integration_test)
    endif()
  else()
    message(STATUS "librga not found; rga_preprocess_test will not be built")
  endif()
endif()

find_program(CLANG_TIDY_EXE clang-tidy)
if(CLANG_TIDY_EXE)
  add_custom_target(lint-clang-tidy
    COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR}
      ${VISION_LINT_SOURCES}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running clang-tidy on vision sources"
  )
else()
  message(STATUS "clang-tidy not found: lint-clang-tidy target will be unavailable")
endif()

find_program(CPPCHECK_EXE cppcheck)
if(CPPCHECK_EXE)
  add_custom_target(lint-cppcheck
    COMMAND ${CPPCHECK_EXE}
      --enable=warning,style,performance,portability
      --std=c++17
      --inline-suppr
      --suppress=missingIncludeSystem
      -I ${CMAKE_SOURCE_DIR}/include
      ${VISION_LINT_SOURCES}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running cppcheck on vision sources"
  )
else()
  message(STATUS "cppcheck not found: lint-cppcheck target will be unavailable")
endif()
