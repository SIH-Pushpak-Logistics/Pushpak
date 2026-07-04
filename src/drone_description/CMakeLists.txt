# 1. The absolute requirement. Must be line 1.
cmake_minimum_required(VERSION 3.8)

# 2. Declare the package name
project(drone_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. Find the ROS 2 build macros
find_package(ament_cmake REQUIRED)

# 4. Install your physical assets to the share directory so ROS can find them
install(DIRECTORY
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 5. This MUST be the absolute last line of the file. It generates local_setup.bash.
ament_package()