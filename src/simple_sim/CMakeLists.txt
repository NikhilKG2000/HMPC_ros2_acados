cmake_minimum_required(VERSION 3.8)
project(simple_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
  -fstack-protector
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneHovergamesControl.msg"
  DEPENDENCIES
    std_msgs
)

add_library(${PROJECT_NAME}_utils
  src/instrumentation_timer.cpp
  src/ros_timer_handler.cpp
)

set_target_properties(${PROJECT_NAME}_utils
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME}_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_utils
  ament_index_cpp
  rcl
  rclcpp
)

add_executable(hovergames_model_node
  src/hovergames_model_node.cpp
)

target_link_libraries(hovergames_model_node
  ${PROJECT_NAME}_utils
)

ament_target_dependencies(hovergames_model_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(hovergames_model_node
  ${cpp_typesupport_target}
)

install(
  TARGETS ${PROJECT_NAME}_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS hovergames_model_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY profiling
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ament_index_cpp
  rcl
  rclcpp
  rosidl_default_runtime
)

ament_package()