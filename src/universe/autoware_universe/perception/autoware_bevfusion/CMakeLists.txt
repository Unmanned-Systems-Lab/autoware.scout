cmake_minimum_required(VERSION 3.14)
project(autoware_bevfusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_compile_options(-Wno-deprecated-declarations)

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
  message("TensorRT is NOT available")
  set(TRT_AVAIL OFF)
endif()

# set flags for spconv availability
option(SPCONV_AVAIL "spconv available" OFF)
# try to find spconv
find_package(cumm)
find_package(spconv)
if(${cumm_FOUND} AND ${spconv_FOUND})
  message(STATUS "spconv is available!")
  set(SPCONV_AVAIL ON)
else()
  message(WARNING "spconv is NOT available")
  set(SPCONV_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND SPCONV_AVAIL)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
  )

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  endif()

  # Find Eigen3
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)

  add_definitions("-DTV_CUDA")

  # cSpell:ignore expt
  list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 1675 --extended-lambda")
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

  cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
    lib/postprocess/circle_nms_kernel.cu
    lib/postprocess/postprocess_kernel.cu
    lib/preprocess/preprocess_kernel.cu
  )

  target_link_libraries(${PROJECT_NAME}_cuda_lib
    spconv::spconv
  )

  target_include_directories(${PROJECT_NAME}_cuda_lib
    SYSTEM PUBLIC
    ${autoware_cuda_utils_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    lib/detection_class_remapper.cpp
    lib/postprocess/non_maximum_suppression.cpp
    lib/preprocess/voxel_generator.cpp
    lib/preprocess/pointcloud_densification.cpp
    lib/preprocess/precomputed_features.cpp
    lib/ros_utils.cpp
    lib/bevfusion_trt.cpp
  )

  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
    TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
  )

  target_link_libraries(${PROJECT_NAME}_lib
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${PROJECT_NAME}_cuda_lib
  )

  # To suppress unknown-pragmas error. The root-cause is CUB library in CUDA 11.6.
  # This issue was fixed by https://github.com/NVIDIA/cub/commit/7d608bf1dc14553e2fb219eabeed80b76621b6fe
  target_include_directories(${PROJECT_NAME}_lib
    SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${autoware_point_types_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/bevfusion_node.cpp
    src/bevfusion_node_utils.cpp
  )

  target_link_libraries(${PROJECT_NAME}_component
    ${PROJECT_NAME}_lib
  )

  rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "autoware::bevfusion::BEVFusionNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  install(
    TARGETS ${PROJECT_NAME}_cuda_lib
    DESTINATION lib
  )

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      config
  )

else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
  )
endif()
