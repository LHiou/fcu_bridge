cmake_minimum_required(VERSION 3.16)
project(fcu_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(conversion STATIC src/conversion.cpp)
target_include_directories(conversion PUBLIC include)
ament_target_dependencies(conversion rclcpp geometry_msgs nav_msgs trajectory_msgs mavros_msgs quadrotor_msgs std_msgs)

add_executable(bridge src/bridge.cpp)
target_link_libraries(bridge conversion)
ament_target_dependencies(bridge std_srvs)
install(TARGETS bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
