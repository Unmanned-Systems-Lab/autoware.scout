cmake_minimum_required(VERSION 3.17)
project(autoware_tensorrt_yolox)

find_package(autoware_tensorrt_common)
if(NOT ${autoware_tensorrt_common_FOUND})
  message(WARNING "The autoware_tensorrt_common package is not found. Please check its dependencies.")
  return()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

# TODO(amadeuszsz): Remove -Wno-deprecated-declarations once removing implicit quantization
add_compile_options(-Wno-deprecated-declarations)

find_package(OpenCV REQUIRED)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

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
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##########
# tensorrt_yolox
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
  src/label.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

if(TRT_AVAIL AND CUDA_AVAIL)
  # Officially, add_library supports .cu file compilation.
  # However, as of cmake 3.22.1, it seems to fail compilation because compiler flags for
  # C++ are directly passed to nvcc (they are originally space separated
  # but nvcc assume comma separated as argument of `-Xcompiler` option).
  # That is why `cuda_add_library` is used here.

  execute_process(
    COMMAND ${CUDA_NVCC_EXECUTABLE} --list-gpu-arch
    OUTPUT_VARIABLE CUDA_SUPPORTED_GPU_ARCHS_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  string(REPLACE "\n" ";" CUDA_SUPPORTED_GPU_ARCHS "${CUDA_SUPPORTED_GPU_ARCHS_RAW}")

  foreach(cuda_arch IN ITEMS 86 87 89 90 101 110 120)
    if("compute_${cuda_arch}" IN_LIST CUDA_SUPPORTED_GPU_ARCHS)
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_${cuda_arch},code=sm_${cuda_arch}")
    endif()
  endforeach()

  if("compute_120" IN_LIST CUDA_SUPPORTED_GPU_ARCHS)
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=compute_120")
  elseif("compute_90" IN_LIST CUDA_SUPPORTED_GPU_ARCHS)
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_90,code=compute_90")
  endif()

  cuda_add_library(${PROJECT_NAME}_gpu_preprocess
    SHARED
    src/preprocess.cu
  )

  target_include_directories(${PROJECT_NAME}_gpu_preprocess PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${CUDA_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}
    ${autoware_tensorrt_common_LIBRARIES}
    ${PROJECT_NAME}_gpu_preprocess
  )

  install(
    TARGETS ${PROJECT_NAME}_gpu_preprocess
    LIBRARY DESTINATION lib
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${autoware_tensorrt_common_LIBRARIES}
  )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

ament_auto_add_library(yolox_single_image_inference_node SHARED
  src/yolox_single_image_inference_node.cpp
)

ament_target_dependencies(yolox_single_image_inference_node
  OpenCV
)

target_link_libraries(yolox_single_image_inference_node
  ${PROJECT_NAME}
  stdc++fs
)

target_compile_definitions(yolox_single_image_inference_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(yolox_single_image_inference_node
  PLUGIN "autoware::tensorrt_yolox::YoloXSingleImageInferenceNode"
  EXECUTABLE yolox_single_image_inference
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/tensorrt_yolox_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::tensorrt_yolox::TrtYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
