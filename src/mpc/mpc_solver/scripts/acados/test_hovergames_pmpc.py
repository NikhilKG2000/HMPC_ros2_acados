#!/usr/bin/env python3
"""Generate and smoke-test the standalone HoverGames PMPC acados solver."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from acados_template import AcadosOcpSolver

from generate_hovergames_pmpc import (
    DT,
    MODEL_NAME,
    N,
    NU,
    NX,
    create_ocp,
    default_parameter_vector,
    package_root,
)


def main() -> None:
    if "ACADOS_SOURCE_DIR" not in os.environ:
        raise RuntimeError("ACADOS_SOURCE_DIR is not set")

    output_directory = package_root() / "acados_generated/hovergames_pmpc"
    output_directory.mkdir(parents=True, exist_ok=True)
    json_name = f"{MODEL_NAME}_acados_ocp.json"
    ocp = create_ocp(output_directory)

    old_cwd = Path.cwd()
    try:
        os.chdir(output_directory)
        solver = AcadosOcpSolver(ocp, json_file=json_name)
    finally:
        os.chdir(old_cwd)

    parameters = default_parameter_vector()
    x_hover = np.zeros(NX)
    x_hover[2] = 1.0
    x_hover[9] = 9.81
    u_hover = np.array([0.0, 0.0, 0.0, 9.81])

    solver.constraints_set(0, "lbx", x_hover)
    solver.constraints_set(0, "ubx", x_hover)
    for stage in range(N + 1):
        solver.set(stage, "p", parameters)
        solver.set(stage, "x", x_hover)
        if stage < N:
            solver.set(stage, "u", u_hover)

    status = solver.solve()
    u0 = solver.get(0, "u")
    x_terminal = solver.get(N, "x")
    terminal_velocity_norm = np.linalg.norm(x_terminal[3:6])

    print(f"status: {status} (0 means success)")
    print(f"horizon: N={N}, dt={DT}, duration={N * DT} s")
    print(f"u0: {u0}")
    print(f"terminal position: {x_terminal[:3]}")
    print(f"terminal velocity norm: {terminal_velocity_norm:.3e}")
    print(f"terminal thrust: {x_terminal[9]:.8f}")

    if status != 0:
        raise SystemExit("PMPC smoke test failed: acados did not return success")
    if not np.all(np.isfinite(u0)) or not np.all(np.isfinite(x_terminal)):
        raise SystemExit("PMPC smoke test failed: non-finite solution")
    if terminal_velocity_norm > 1.0e-5:
        raise SystemExit("PMPC smoke test failed: terminal state is not steady")

    print("PMPC smoke test passed")


if __name__ == "__main__":
    main()
