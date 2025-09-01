cmake_minimum_required(VERSION 3.8)
project(refboard_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include)

# Create the refboard_base library
add_library(refboard_base SHARED
  src/mks_can_interface.cpp
  src/mks_protocol.cpp
)

target_compile_features(refboard_base PRIVATE cxx_std_17)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install library
install(TARGETS refboard_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build test program
add_executable(test_mks_can test/test_mks_can.cpp)
target_link_libraries(test_mks_can refboard_base)
target_compile_features(test_mks_can PRIVATE cxx_std_17)

# Build simple test program
add_executable(test_motor_simple test/test_motor_simple.cpp)
target_link_libraries(test_motor_simple refboard_base)
target_compile_features(test_motor_simple PRIVATE cxx_std_17)

# Install test programs
install(TARGETS test_mks_can test_motor_simple
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
