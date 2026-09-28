cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_plugins)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_compile_options(-Wno-deprecated-declarations)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
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
  foreach(cuda_target_include_dir
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include"
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/include"
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/sbsa-linux/include")
    if(EXISTS "${cuda_target_include_dir}/cublas_v2.h")
      list(APPEND CUDA_INCLUDE_DIRS "${cuda_target_include_dir}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES CUDA_INCLUDE_DIRS)
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
  find_package(tensorrt_cmake_module REQUIRED)
  find_package(TENSORRT)
  if(TENSORRT_VERSION VERSION_LESS 10.0)
    message(WARNING "Unsupported version TensorRT ${TENSORRT_VERSION} detected. This package requires TensorRT 10.0 or later.")
    return()
  endif()

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
  )

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  endif()

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

  function(append_cuda_gencode_if_supported arch)
    if("compute_${arch}" IN_LIST CUDA_SUPPORTED_GPU_ARCHS)
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_${arch},code=sm_${arch}")
      set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" PARENT_SCOPE)
    endif()
  endfunction()

  foreach(arch IN ITEMS 86 87 89 90 101 110 120)
    append_cuda_gencode_if_supported(${arch})
  endforeach()

  if("compute_120" IN_LIST CUDA_SUPPORTED_GPU_ARCHS)
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=compute_120")
  elseif("compute_90" IN_LIST CUDA_SUPPORTED_GPU_ARCHS)
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_90,code=compute_90")
  endif()

  cuda_add_library(cuda_ops SHARED
    src/argsort_ops/argsort.cu
    src/bev_ops/bev_pool_cuda.cu
    src/scatter_ops/segment_csr.cu
    src/unique_ops/unique.cu
    src/multi_scale_deform_attn_ops/ms_deform_attn_kernel.cu
    src/rotate_ops/rotate_kernel.cu
    src/select_and_pad_ops/select_and_pad_kernel.cu
  )

  add_library(${PROJECT_NAME} SHARED
    src/argsort_plugin.cpp
    src/argsort_plugin_creator.cpp
    src/quick_cumsum_cuda_plugin.cpp
    src/quick_cumsum_cuda_plugin_creator.cpp
    src/get_indices_pairs_implicit_gemm_plugin_creator.cpp
    src/get_indices_pairs_implicit_gemm_plugin.cpp
    src/get_indices_pairs_plugin_creator.cpp
    src/get_indices_pairs_plugin.cpp
    src/implicit_gemm_plugin_creator.cpp
    src/implicit_gemm_plugin.cpp
    src/indice_conv_plugin_creator.cpp # cSpell:ignore indice
    src/indice_conv_plugin.cpp
    src/multi_scale_deformable_attention_plugin.cpp
    src/multi_scale_deformable_attention_plugin_creator.cpp
    src/rotate_plugin.cpp
    src/rotate_plugin_creator.cpp
    src/segment_csr_plugin_creator.cpp
    src/segment_csr_plugin.cpp
    src/select_and_pad_plugin.cpp
    src/select_and_pad_plugin_creator.cpp
    src/unique_plugin.cpp
    src/unique_plugin_creator.cpp
    src/plugin_registration.cpp
    src/plugin_utils.cpp
  )

  target_compile_definitions(${PROJECT_NAME} PRIVATE _GLIBCXX_USE_CXX11_ABI=1)

  target_link_libraries(${PROJECT_NAME} PRIVATE
      ${NVINFER}
      CUDA::cudart
      cuda_ops
      spconv::spconv
  )

  install(
    TARGETS ${PROJECT_NAME}
    DESTINATION share/${PROJECT_NAME}/plugins
  )

  install(
    TARGETS cuda_ops
    LIBRARY DESTINATION lib
  )

  ament_auto_package()
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
endif()
