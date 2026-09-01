cmake_minimum_required(VERSION 3.16)
project(stage3_convex_mpc)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eiquadprog REQUIRED)

add_executable(convex_mpc_node src/convex_mpc_node.cpp)
target_include_directories(convex_mpc_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(convex_mpc_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)
target_link_libraries(convex_mpc_node eiquadprog::eiquadprog)

install(TARGETS convex_mpc_node DESTINATION lib/${PROJECT_NAME})

ament_package()
