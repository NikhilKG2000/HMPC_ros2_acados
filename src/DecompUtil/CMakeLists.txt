cmake_minimum_required(VERSION 3.8)
project(decomp_util)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(DECOMP_OLD FALSE)
set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mpc_tools REQUIRED)

set(LIBRARY_SOURCES
  src/decomp_basis/data_utils.cpp
  src/decomp_geometry/ellipsoid.cpp
  src/decomp_geometry/geometric_utils.cpp
  src/decomp_geometry/polyhedron.cpp
  src/decomp_base.cpp
  src/ellipsoid_decomp.cpp
  src/iterative_decomp.cpp
  src/line_segment.cpp
  src/seed_decomp.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_SOURCES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall
    -Wno-deprecated-declarations
    -O3
)

if(DECOMP_OLD)
  target_compile_definitions(${PROJECT_NAME} PRIVATE DECOMP_OLD)
endif()

ament_target_dependencies(${PROJECT_NAME}
  mpc_tools
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  Eigen3
  mpc_tools
)

ament_package()