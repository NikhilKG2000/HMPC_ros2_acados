cmake_minimum_required(VERSION 3.8)
project(mpc_base)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mpc_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_dependencies(
  Eigen3
  mpc_msgs
)

ament_package()