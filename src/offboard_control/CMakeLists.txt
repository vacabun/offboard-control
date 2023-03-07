cmake_minimum_required(VERSION 3.5)
project(offboard_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
add_executable(offboard_position_control 
  src/offboard_position_control.cpp)

target_include_directories(offboard_position_control PUBLIC include)

ament_target_dependencies(offboard_position_control rclcpp ament_index_cpp mavros mavros_msgs)

install(TARGETS
  offboard_position_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
