cmake_minimum_required(VERSION 3.8)
project(turtlesim_border_color)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

install(
  DIRECTORY  
  launch 
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/border_color_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
