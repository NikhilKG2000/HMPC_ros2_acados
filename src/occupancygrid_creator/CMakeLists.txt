cmake_minimum_required(VERSION 3.8)
project(occupancygrid_creator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
)

find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
  src/occupancygrid_creator.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  gazebo_msgs
  geometry_msgs
  mpc_tools
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(occupancygrid_creator_node
  src/occupancygrid_creator_node.cpp
)

target_link_libraries(occupancygrid_creator_node
  ${PROJECT_NAME}
)

ament_target_dependencies(occupancygrid_creator_node
  rclcpp
)

install(
  TARGETS
    ${PROJECT_NAME}
    occupancygrid_creator_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  FILES
    include/${PROJECT_NAME}/occupancygrid_creator.h
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config
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
  gazebo_msgs
  geometry_msgs
  mpc_tools
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

ament_package()