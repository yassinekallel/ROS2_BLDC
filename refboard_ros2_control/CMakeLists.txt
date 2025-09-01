cmake_minimum_required(VERSION 3.8)
project(refboard_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(refboard_base REQUIRED)

include_directories(include)

# Create the hardware interface plugin
add_library(refboard_ros2_control_plugin SHARED
  src/refboard_hardware_interface.cpp
)

# Link with refboard_base
target_link_libraries(refboard_ros2_control_plugin refboard_base)

ament_target_dependencies(refboard_ros2_control_plugin
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  refboard_base
)

pluginlib_export_plugin_description_file(hardware_interface refboard_hardware_interface.xml)

target_compile_features(refboard_ros2_control_plugin PRIVATE cxx_std_17)

install(TARGETS refboard_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES refboard_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
