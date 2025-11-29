cmake_minimum_required(VERSION 3.5)
project(competition_test_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CURSES_NEED_WIDE TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Curses REQUIRED)
find_package(handyman_msgs REQUIRED)
find_package(interactive_cleanup_msgs REQUIRED)
find_package(human_navigation_msgs REQUIRED)

add_executable(teleop_key src/teleop_key.cpp)
ament_target_dependencies(teleop_key rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs handyman_msgs interactive_cleanup_msgs human_navigation_msgs)
target_link_libraries(teleop_key ${CURSES_LIBRARIES})
target_include_directories(teleop_key PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CURSES_INCLUDE_DIR})

install(
  TARGETS teleop_key
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

