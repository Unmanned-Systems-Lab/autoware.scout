cmake_minimum_required(VERSION 3.5)
project(autoware_tensorrt_classifier)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wno-write-strings -fopenmp -Wall")

find_package(autoware_cmake REQUIRED)
autoware_package()

# TODO(amadeuszsz): Remove -Wno-deprecated-declarations once removing implicit quantization
add_compile_options(-Wno-deprecated-declarations)

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
find_package(TENSORRT)
find_package(OpenCV REQUIRED)

if(NOT (${CUDA_FOUND} AND ${TENSORRT_FOUND}))
  message(WARNING "cuda, tensorrt libraries are not found")
  return()
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_classifier.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
)

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

target_include_directories(${PROJECT_NAME}_gpu_preprocess
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${TENSORRT_LIBRARIES}
  ${PROJECT_NAME}_gpu_preprocess
  stdc++fs
)

install(
  TARGETS ${PROJECT_NAME}_gpu_preprocess
  LIBRARY DESTINATION lib
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

list(APPEND ${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(CUDA)
ament_export_dependencies(tensorrt_cmake_module)
ament_export_dependencies(TENSORRT)

ament_auto_package()
