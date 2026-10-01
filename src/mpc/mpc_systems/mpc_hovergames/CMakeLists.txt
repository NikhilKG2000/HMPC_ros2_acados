cmake_minimum_required(VERSION 3.8)
project(mpc_hovergames)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_base REQUIRED)
find_package(mpc_core REQUIRED)
find_package(mpc_modules REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(mpc_solver REQUIRED)
find_package(mpc_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_sim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hovergames_drone_interface.cpp
  src/hovergames_simplesim_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wno-deprecated-declarations
  -O3
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  SIMPLESIM
  NDEBUG
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  mpc_base
  mpc_core
  mpc_modules
  mpc_msgs
  mpc_tools
  nav_msgs
  rclcpp
  sensor_msgs
  simple_sim
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(mpc_control_node_2_layer
  src/mpc_control_node_2_layer.cpp
)

target_compile_features(mpc_control_node_2_layer PUBLIC cxx_std_14)

target_compile_options(mpc_control_node_2_layer PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
  -O3
)

target_link_libraries(mpc_control_node_2_layer
  ${PROJECT_NAME}
)

ament_target_dependencies(mpc_control_node_2_layer
  mpc_base
  mpc_core
  mpc_modules
  mpc_solver
  mpc_tools
  rclcpp
  rcl_interfaces
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS mpc_control_node_2_layer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    include/mpc_hovergames/hovergames_drone_interface.h
    include/mpc_hovergames/hovergames_simplesim_interface.h
  DESTINATION include/mpc_hovergames
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  mpc_base
  mpc_core
  mpc_modules
  mpc_msgs
  mpc_tools
  nav_msgs
  rclcpp
  sensor_msgs
  simple_sim
  std_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()
