cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Core Build Tools
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the C++ and Python bindings for your custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OpticalFlow.msg"
  DEPENDENCIES std_msgs
)

ament_package()