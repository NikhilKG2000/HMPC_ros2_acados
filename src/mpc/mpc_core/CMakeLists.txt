cmake_minimum_required(VERSION 3.8)
project(mpc_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mpc_base REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(mpc_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(LIBRARY_SOURCES
  src/functions/update_mpc_configuration.cpp
  src/functions/error_function.cpp
  src/interface.cpp
  src/controller.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${LIBRARY_SOURCES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  mpc_base
  mpc_msgs
  mpc_tools
  rclcpp
  sensor_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_export_dependencies(
  Eigen3
  mpc_base
  mpc_msgs
  mpc_tools
  rclcpp
  sensor_msgs
)

ament_package()