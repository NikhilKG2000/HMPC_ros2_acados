cmake_minimum_required(VERSION 3.8)
project(mpc_solver LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mpc_base REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
  message(FATAL_ERROR
    "ACADOS_SOURCE_DIR is not set. Export it before running colcon build.")
endif()

set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
set(ACADOS_INCLUDE_DIR "${ACADOS_SOURCE_DIR}/include")
set(ACADOS_LIBRARY_DIR "${ACADOS_SOURCE_DIR}/lib")
set(TMPC_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/hovergames_tmpc/c_generated_code")
set(PMPC_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/hovergames_pmpc/c_generated_code")

if(NOT EXISTS "${TMPC_GENERATED_DIR}/acados_solver_hovergames_tmpc.h")
  message(FATAL_ERROR
    "The generated HoverGames TMPC solver was not found. Run:\n"
    "python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py")
endif()

if(NOT EXISTS "${PMPC_GENERATED_DIR}/acados_solver_hovergames_pmpc.h")
  message(FATAL_ERROR
    "The generated HoverGames PMPC solver was not found. Run:\n"
    "python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py")
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/systems/hovergames/pmpc_settings.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/systems/hovergames/tmpc_settings.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/hmpc_settings.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/include/offline_computations/offline_computations.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/include/offline_computations/mpc-sdp/offline_comps_tracking.mat"
)

execute_process(
  COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/hmpc_settings.py"
  RESULT_VARIABLE HMPC_SETTINGS_CHECK_RESULT
  OUTPUT_VARIABLE HMPC_SETTINGS_CHECK_OUTPUT
  ERROR_VARIABLE HMPC_SETTINGS_CHECK_ERROR
)

if(NOT HMPC_SETTINGS_CHECK_RESULT EQUAL 0)
  message(FATAL_ERROR
    "${HMPC_SETTINGS_CHECK_OUTPUT}${HMPC_SETTINGS_CHECK_ERROR}")
endif()

find_library(ACADOS_LIBRARY acados
  PATHS "${ACADOS_LIBRARY_DIR}" NO_DEFAULT_PATH)
find_library(BLASFEO_LIBRARY blasfeo
  PATHS "${ACADOS_LIBRARY_DIR}" NO_DEFAULT_PATH)
find_library(HPIPM_LIBRARY hpipm
  PATHS "${ACADOS_LIBRARY_DIR}" NO_DEFAULT_PATH)

if(NOT ACADOS_LIBRARY OR NOT BLASFEO_LIBRARY OR NOT HPIPM_LIBRARY)
  message(FATAL_ERROR
    "Could not find libacados, libblasfeo and libhpipm in ${ACADOS_LIBRARY_DIR}")
endif()

set(TMPC_GENERATED_SOURCES
  ${TMPC_GENERATED_DIR}/acados_solver_hovergames_tmpc.c

  ${TMPC_GENERATED_DIR}/hovergames_tmpc_model/hovergames_tmpc_dyn_disc_phi_fun.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_model/hovergames_tmpc_dyn_disc_phi_fun_jac.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_model/hovergames_tmpc_dyn_disc_phi_fun_jac_hess.c

  ${TMPC_GENERATED_DIR}/hovergames_tmpc_constraints/hovergames_tmpc_constr_h_fun.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_constraints/hovergames_tmpc_constr_h_fun_jac_uxt_zt.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_constraints/hovergames_tmpc_constr_h_fun_jac_uxt_zt_hess.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_constraints/hovergames_tmpc_constr_h_e_fun.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_constraints/hovergames_tmpc_constr_h_e_fun_jac_uxt_zt.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_constraints/hovergames_tmpc_constr_h_e_fun_jac_uxt_zt_hess.c

  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_0_fun.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_0_fun_jac.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_0_fun_jac_hess.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_fun.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_fun_jac.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_fun_jac_hess.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_e_fun.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_e_fun_jac.c
  ${TMPC_GENERATED_DIR}/hovergames_tmpc_cost/hovergames_tmpc_cost_ext_cost_e_fun_jac_hess.c
)

set(PMPC_GENERATED_SOURCES
  ${PMPC_GENERATED_DIR}/acados_solver_hovergames_pmpc.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_model/hovergames_pmpc_dyn_disc_phi_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_model/hovergames_pmpc_dyn_disc_phi_fun_jac.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_model/hovergames_pmpc_dyn_disc_phi_fun_jac_hess.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_0_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_0_fun_jac_uxt_zt.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_0_fun_jac_uxt_zt_hess.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_fun_jac_uxt_zt.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_fun_jac_uxt_zt_hess.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_e_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_e_fun_jac_uxt_zt.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_constraints/hovergames_pmpc_constr_h_e_fun_jac_uxt_zt_hess.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_0_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_0_fun_jac.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_0_fun_jac_hess.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_fun_jac.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_fun_jac_hess.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_e_fun.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_e_fun_jac.c
  ${PMPC_GENERATED_DIR}/hovergames_pmpc_cost/hovergames_pmpc_cost_ext_cost_e_fun_jac_hess.c
)

add_library(hovergames_tmpc_acados_generated SHARED
  ${TMPC_GENERATED_SOURCES}
)

add_library(hovergames_pmpc_acados_generated SHARED
  ${PMPC_GENERATED_SOURCES}
)

target_compile_features(hovergames_tmpc_acados_generated PRIVATE c_std_99)
target_compile_features(hovergames_pmpc_acados_generated PRIVATE c_std_99)

target_include_directories(hovergames_tmpc_acados_generated PRIVATE
  ${TMPC_GENERATED_DIR}
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
)

target_include_directories(hovergames_pmpc_acados_generated PRIVATE
  ${PMPC_GENERATED_DIR}
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
)

target_link_libraries(hovergames_tmpc_acados_generated PUBLIC
  ${ACADOS_LIBRARY}
  ${BLASFEO_LIBRARY}
  ${HPIPM_LIBRARY}
  m
)

target_link_libraries(hovergames_pmpc_acados_generated PUBLIC
  ${ACADOS_LIBRARY}
  ${BLASFEO_LIBRARY}
  ${HPIPM_LIBRARY}
  m
)

add_library(${PROJECT_NAME} SHARED
  src/acados/acados_solver.cpp
  src/core/helper_functions.cpp
  src/hovergames/hovergames_solver_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
  -O3
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PMPC_GENERATED_DIR}
    ${TMPC_GENERATED_DIR}
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  hovergames_pmpc_acados_generated
  hovergames_tmpc_acados_generated
)

ament_target_dependencies(${PROJECT_NAME}
  mpc_base
)

add_executable(test_hovergames_tmpc_adapter
  test/test_hovergames_tmpc_adapter.cpp
)

add_executable(test_hovergames_pmpc_adapter
  test/test_hovergames_pmpc_adapter.cpp
)

target_link_libraries(test_hovergames_tmpc_adapter
  ${PROJECT_NAME}
)

target_link_libraries(test_hovergames_pmpc_adapter
  ${PROJECT_NAME}
)

install(
  TARGETS
    hovergames_pmpc_acados_generated
    hovergames_tmpc_acados_generated
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The smoke tests are runnable executables, not linkable package targets.
# Install them for `ros2 run` without adding them to the exported target set.
install(
  TARGETS
    test_hovergames_pmpc_adapter
    test_hovergames_tmpc_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 mpc_base)

ament_package()
