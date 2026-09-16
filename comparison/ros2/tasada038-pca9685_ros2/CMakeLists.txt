cmake_minimum_required(VERSION 3.8)
project(pca9685_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Add executable
add_library(JHPWMPCA9685 src/JHPWMPCA9685.cpp)
add_executable(controller src/pca9685_node.cpp src/pca9685_component.cpp)
ament_target_dependencies(controller
  rclcpp
  sensor_msgs
)
target_link_libraries(controller JHPWMPCA9685 i2c)

# Install Cpp executables
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
