cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# add executable
add_executable(my_test_node src/my_test_node.cpp)
add_executable(motor_publisher src/motor_publisher.cpp)
add_executable(smart_motor src/smart_motor.cpp)

# link dependencies
ament_target_dependencies(my_test_node rclcpp)
ament_target_dependencies(motor_publisher rclcpp std_msgs)
ament_target_dependencies(smart_motor rclcpp std_msgs)

# install executable
install(TARGETS
  my_test_node
  motor_publisher
  smart_motor
  DESTINATION lib/${PROJECT_NAME}
)

# register package
ament_package()