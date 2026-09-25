cmake_minimum_required(VERSION 3.14)
project(autoware_diffusion_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Options
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# Find CUDA
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  foreach(cuda_target_include_dir
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include"
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/include"
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/sbsa-linux/include")
    if(EXISTS "${cuda_target_include_dir}/cublas_v2.h")
      list(APPEND CUDA_INCLUDE_DIRS "${cuda_target_include_dir}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES CUDA_INCLUDE_DIRS)
endif()
if(CUDA_FOUND)
set(CUDA_TARGET_LIBRARY_DIRS
    "${CUDA_TOOLKIT_ROOT_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib"
    "${CUDA_TOOLKIT_ROOT_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib/stubs"
    "${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/lib"
    "${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/lib/stubs"
    "${CUDA_TOOLKIT_ROOT_DIR}/targets/sbsa-linux/lib"
    "${CUDA_TOOLKIT_ROOT_DIR}/targets/sbsa-linux/lib/stubs"
  )
  list(APPEND CUDA_LIBRARY_DIRS ${CUDA_TARGET_LIBRARY_DIRS})
  list(REMOVE_DUPLICATES CUDA_LIBRARY_DIRS)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
    ${CUDA_TARGET_LIBRARY_DIRS}
  )
  if(NOT CUDA_curand_LIBRARY)
    find_library(CUDA_curand_LIBRARY curand HINTS
      ${CUDA_TOOLKIT_ROOT_DIR}/lib64
      ${CUDA_TOOLKIT_ROOT_DIR}/lib
      ${CUDA_TARGET_LIBRARY_DIRS}
    )
  endif()
  if(CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message(WARNING "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# Find TensorRT libraries
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message(WARNING "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL)
  # Add compile option to suppress deprecation warnings (optional)
  add_compile_options(-Wno-deprecated-declarations)

  # Suppress narrowing conversion warnings from TensorRT headers
  add_compile_options(-Wno-narrowing)

  # Add your component library
  ament_auto_add_library(autoware_diffusion_planner_component SHARED
    src/diffusion_planner_node.cpp
    src/conversion/lanelet.cpp
    src/conversion/agent.cpp
    src/postprocessing/postprocessing_utils.cpp
    src/preprocessing/lane_segments.cpp
    src/preprocessing/preprocessing_utils.cpp
    src/preprocessing/traffic_signals.cpp
    src/utils/utils.cpp
    src/utils/marker_utils.cpp
    src/postprocessing/turn_indicator_manager.cpp
    src/inference/tensorrt_inference.cpp
  )

  target_include_directories(autoware_diffusion_planner_component PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${autoware_cuda_utils_INCLUDE_DIRS}
  )
  target_link_libraries(autoware_diffusion_planner_component
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
  )

  rclcpp_components_register_node(autoware_diffusion_planner_component
    PLUGIN "autoware::diffusion_planner::DiffusionPlanner"
    EXECUTABLE autoware_diffusion_planner_node
  )

  if(BUILD_TESTING)
    ament_add_ros_isolated_gtest(test_diffusion_planner
      test/agent_test.cpp
      test/lanelet_test.cpp
      test/lane_segments_test.cpp
      test/pre_processing_utils_test.cpp
      test/postprocessing_utils_test.cpp
      test/utils_test.cpp
      test/marker_utils_test.cpp
      test/lanelet_integration_test.cpp
      test/postprocessing_utils_edge_case_test.cpp
      test/preprocessing_utils_edge_case_test.cpp
      test/agent_edge_case_test.cpp
      test/lanelet_edge_case_test.cpp
      test/diffusion_planner_integration_test.cpp)

    target_link_libraries(test_diffusion_planner autoware_diffusion_planner_component)
    ament_target_dependencies(test_diffusion_planner
      autoware_test_utils
    )
  endif()
endif()
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test_map
)
