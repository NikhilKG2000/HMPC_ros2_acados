cmake_minimum_required(VERSION 3.8)
project(mpc_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(DEBUGGING FALSE)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

set(LIBRARY_SOURCES
  src/eigen_to_std.cpp
  src/ros_visuals.cpp
  src/data_saver_json.cpp
  src/benchmarker.cpp
  src/robot_region.cpp
  src/instrumentation_timer.cpp
  src/ROS_timer_handler.cpp
  src/ThreadPool.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_SOURCES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -fstack-protector
    -Wall
    -O3
    -Wno-deprecated-declarations
)

if(DEBUGGING)
  target_compile_definitions(${PROJECT_NAME} PRIVATE OLD_DECOMP)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE NDEBUG)
endif()

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    $<BUILD_INTERFACE:Boost::filesystem>
    $<BUILD_INTERFACE:Boost::thread>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY profiling recorded_data
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ament_index_cpp
  Eigen3
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  visualization_msgs
)

ament_package()