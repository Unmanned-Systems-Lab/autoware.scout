cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_fine_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_compile_options(-Wno-deprecated-declarations)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

find_package(OpenCV REQUIRED)

# set flags for CUDA availability
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
  set(CUDA_AVAIL ON)
else()
  message(STATUS "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message(STATUS "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL)
  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/traffic_light_fine_detector_node.cpp
    src/traffic_light_fine_detector_process.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    lib/include
  )

  target_link_libraries(${PROJECT_NAME}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${OpenCV_LIB}
    stdc++fs
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::traffic_light::TrafficLightFineDetectorNode"
    EXECUTABLE traffic_light_fine_detector_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_auto_add_gtest(test_utils
      test/test_utils.cpp
    )
  endif()

else()
  message(STATUS "TrafficLightFineDetector won't be built, CUDA and/or TensorRT were not found.")
  # to avoid launch file missing without a gpu
  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )
endif()
