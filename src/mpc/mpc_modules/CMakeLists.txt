cmake_minimum_required(VERSION 3.8)
project(mpc_modules)

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
  -O3
)

# Use the current DecompUtil map-preprocessing behavior.
set(DECOMP_OLD FALSE)

if(DECOMP_OLD)
  add_compile_definitions(DECOMP_OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(decomp_util REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_base REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(mpc_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

set(LIBRARY_SOURCES
  src/loader/module_handler.cpp
  src/objectives/goal_oriented.cpp
  src/objectives/reference_trajectory.cpp
  src/constraints/static_polyhedron_constraints.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${LIBRARY_SOURCES}
)

# Compile every available HMPC module. ModuleHandler still instantiates
# only the modules requested by the generated solver configuration.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    goal_oriented
    reference_trajectory
    static_polyhedron_constraints
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  decomp_util
  geometry_msgs
  mpc_base
  mpc_msgs
  mpc_tools
  nav_msgs
  rclcpp
  tf2
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
  decomp_util
  Eigen3
  geometry_msgs
  mpc_base
  mpc_msgs
  mpc_tools
  nav_msgs
  OpenCV
  rclcpp
  tf2
)

ament_package()