cmake_minimum_required(VERSION 3.14)
project(autoware_path_sampler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_path_sampler SHARED
  DIRECTORY src
)

# register node
rclcpp_components_register_node(autoware_path_sampler
  PLUGIN "autoware::path_sampler::PathSampler"
  EXECUTABLE path_sampler_exe
)

if(BUILD_TESTING)
  ament_add_gtest(test_ht_path_selection test/test_ht_path_selection.cpp)
  target_link_libraries(test_ht_path_selection autoware_path_sampler)
  target_compile_definitions(test_ht_path_selection PRIVATE SCOUT_WS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../../../..")
endif()

ament_auto_package(
#   INSTALL_TO_SHARE
)
