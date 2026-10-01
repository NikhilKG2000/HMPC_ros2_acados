#!/usr/bin/env python3
"""Generate the acados HoverGames tracking MPC (TMPC) solver.

This reproduces the solver defined by systems/hovergames/tmpc_settings.py
through the existing SolverBase interface.
"""

from __future__ import annotations
from hmpc_settings import (
    load_settings, interval, write_metadata, original_inequalities,
)

import argparse
import os
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


MODEL_NAME = "hovergames_tmpc"

SETTINGS = load_settings("tmpc")

N = SETTINGS.N
DT = interval(SETTINGS)

NX = SETTINGS.model.nx
NU = SETTINGS.model.nu
PARAMETERS = SETTINGS.params.params[0]
N_REFERENCE_PARAMETERS = PARAMETERS["n_par_objectives"]
N_OBSTACLE_PARAMETERS = PARAMETERS["n_par_constraints"]
N_POLYHEDRON_CONSTRAINTS = N_OBSTACLE_PARAMETERS // 3
assert N_OBSTACLE_PARAMETERS == 3 * N_POLYHEDRON_CONSTRAINTS
NP = N_REFERENCE_PARAMETERS + N_OBSTACLE_PARAMETERS


def package_root() -> Path:
    return Path(__file__).resolve().parents[2]


def export_model() -> AcadosModel:
    model = AcadosModel()

    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    p = ca.SX.sym("p", NP)

    # Original HMPC stage-variable ordering: [u; x].
    z = ca.vertcat(u, x)

    next_state = SETTINGS.model.discretize_dynamics(
        z, p, SETTINGS
    )

    model.name = MODEL_NAME
    model.x = x
    model.u = u
    model.p = p
    model.disc_dyn_expr = ca.vertcat(*next_state)

    return model


def default_parameter_vector() -> np.ndarray:
    """Return a safe hover reference with inactive obstacle constraints."""
    p = np.zeros(NP)
    p[3] = 9.81  # thrust_c reference
    p[NU + 9] = 9.81  # thrust state reference

    obstacle_offset = N_REFERENCE_PARAMETERS
    for index in range(N_POLYHEDRON_CONSTRAINTS):
        row = obstacle_offset + 3 * index
        p[row : row + 3] = (1.0, 0.0, 100.0)
    return p


def create_ocp(output_directory: Path) -> AcadosOcp:
    model = export_model()
    x, u, p = model.x, model.u, model.p

    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = str(output_directory / "c_generated_code")

    # Compatibility with both older and current acados_template releases.
    if hasattr(ocp.solver_options, "N_horizon"):
        ocp.solver_options.N_horizon = N
    else:
        ocp.dims.N = N
    ocp.solver_options.tf = N * DT

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    z = ca.vertcat(u, x)

    model.cost_expr_ext_cost = (
        SETTINGS.use_objective(0, z, p, SETTINGS) / DT
    )

    # The original TMPC terminal objective contains no input term.
    terminal_z = ca.vertcat(ca.SX.zeros(NU), x)

    model.cost_expr_ext_cost_e = SETTINGS.use_objective(
        N, terminal_z, p, SETTINGS
    )

    # Original model bounds are ordered as [u, x].
    lower_bounds = np.asarray(SETTINGS.model.lower_bound(), dtype=float)
    upper_bounds = np.asarray(SETTINGS.model.upper_bound(), dtype=float)

    ocp.constraints.idxbu = np.arange(NU)
    ocp.constraints.lbu = lower_bounds[:NU]
    ocp.constraints.ubu = upper_bounds[:NU]

    state_lower = lower_bounds[NU:]
    state_upper = upper_bounds[NU:]

    ocp.constraints.idxbx = np.arange(NX)
    ocp.constraints.lbx = state_lower
    ocp.constraints.ubx = state_upper
    ocp.constraints.idxbx_e = np.arange(NX)
    ocp.constraints.lbx_e = state_lower
    ocp.constraints.ubx_e = state_upper

    # x0 is overwritten on every controller iteration by the C++ adapter.
    x0 = np.zeros(NX)
    x0[2] = 1.0
    x0[9] = 9.81
    ocp.constraints.x0 = x0

    # Use the original module manager's stage allocation and bounds.
    z = ca.vertcat(u, x)
    model.con_h_expr_0, ocp.constraints.lh_0, ocp.constraints.uh_0 = (
        original_inequalities(SETTINGS, 0, z, p)
    )
    model.con_h_expr, ocp.constraints.lh, ocp.constraints.uh = (
        original_inequalities(SETTINGS, 1, z, p)
    )
    model.con_h_expr_e, ocp.constraints.lh_e, ocp.constraints.uh_e = (
        original_inequalities(SETTINGS, N, terminal_z, p)
    )

    ocp.parameter_values = default_parameter_vector()

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 300
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.regularize_method = "CONVEXIFY"
    ocp.solver_options.integrator_type = "DISCRETE"
   
    ocp.solver_options.print_level = 0
    return ocp


def generate(output_directory: Path) -> Path:
    if "ACADOS_SOURCE_DIR" not in os.environ:
        raise RuntimeError("ACADOS_SOURCE_DIR is not set")

    output_directory.mkdir(parents=True, exist_ok=True)
    ocp = create_ocp(output_directory)
    json_name = f"{MODEL_NAME}_acados_ocp.json"

    old_cwd = Path.cwd()
    try:
        os.chdir(output_directory)
        AcadosOcpSolver(ocp, json_file=json_name)
    finally:
        os.chdir(old_cwd)
    write_metadata("tmpc", output_directory)
    return output_directory / json_name


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=package_root() / "acados_generated/hovergames_tmpc",
        help="directory for the generated acados solver",
    )
    args = parser.parse_args()
    json_path = generate(args.output_dir.resolve())
    print(f"Generated {MODEL_NAME} solver")
    print(f"JSON: {json_path}")
    print(f"C code: {args.output_dir.resolve() / 'c_generated_code'}")


if __name__ == "__main__":
    main()
