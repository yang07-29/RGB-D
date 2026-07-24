cmake_minimum_required(VERSION 3.16)
project(rgbd_odometry_ros LANGUAGES NONE)

option(RGBD_ROS_BUILD_CPP "Build the C++17 ROS 2 odometry executable" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
if(RGBD_ROS_BUILD_CPP)
  enable_language(CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(rclcpp REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(cv_bridge REQUIRED)

  set(RGBD_BUILD_APPS OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp ${CMAKE_CURRENT_BINARY_DIR}/rgbd_core)
  add_executable(rgbd_odometry_node src/rgbd_odometry_node.cpp)
  # ament_target_dependencies uses CMake's plain target_link_libraries
  # signature, so keep every link call for this target in the same form.
  target_link_libraries(rgbd_odometry_node rgbd_core)
  ament_target_dependencies(
    rgbd_odometry_node
    rclcpp builtin_interfaces sensor_msgs nav_msgs geometry_msgs tf2_ros message_filters cv_bridge
  )
  if(MSVC)
    target_compile_options(rgbd_odometry_node PRIVATE /W4 /permissive-)
    target_link_libraries(rgbd_odometry_node Psapi)
  else()
    target_compile_options(rgbd_odometry_node PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  install(TARGETS rgbd_odometry_node DESTINATION lib/${PROJECT_NAME})
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/tum_rgbd_publisher scripts/rgbd_odometry_py_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
