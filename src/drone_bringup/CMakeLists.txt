cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

# Install the launch directory so ROS 2 can find swarm_bringup.launch.py
# Install the launch and config directories so ROS 2 can find them
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()