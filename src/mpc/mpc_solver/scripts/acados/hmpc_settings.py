"""Load the original HMPC settings for the acados backend."""

import contextlib
import importlib
import io
import json
from pathlib import Path
import sys


def package_root():
    return Path(__file__).resolve().parents[2]


def load_settings(layer):
    if layer not in ("pmpc", "tmpc"):
        raise ValueError(f"Unknown HMPC layer: {layer}")

    scripts = package_root() / "scripts"

    # Use the same import paths as the original HMPC setup script.
    for directory in (
        scripts,
        scripts / "systems",
        scripts / "include",
    ):
        if str(directory) not in sys.path:
            sys.path.insert(0, str(directory))

    # Suppress the settings files' lengthy initialization printouts.
    with contextlib.redirect_stdout(io.StringIO()):
        return importlib.import_module(
            f"hovergames.{layer}_settings"
        )


def interval(settings):
    return (
        settings.integrator_options["stepsize"]
        * settings.integrator_options["steps"]
    )

def original_inequalities(settings, stage_idx, z, p):
    """Adapt parameter layout and expose the original constraint module output."""
    import casadi as ca
    import numpy as np

    stage = settings.params.params[stage_idx]
    original_p = p
    if p.numel() != stage["n_par"]:
        # TMPC's acados parameter vector omits its compiled weights. Restore
        # their slots before the original module loads obstacle parameters.
        if (not settings.hardcode_weights or
                p.numel() != stage["n_par"] - stage["n_par_weights"]):
            raise ValueError("Unexpected acados/original parameter layout")
        count = stage["n_par_objectives"]
        compiled_weights = ca.DM([
            settings.weights.weight_dict[name]
            for name in stage["par_weights_names"]
        ])
        original_p = ca.vertcat(p[:count], compiled_weights, p[count:])

    settings.params.load_objectives_params(stage_idx, original_p)
    expressions = settings.modules.inequality_constraints(
        stage_idx, z, original_p, settings
    )
    manager = settings.modules.constraint_manager
    lower = np.asarray(manager.lower_bound[stage_idx], dtype=float)
    upper = np.asarray(manager.upper_bound[stage_idx], dtype=float)
    # Retain the existing acados representation of one-sided bounds.
    lower = np.where(np.isneginf(lower), -1.0e15, lower)
    upper = np.where(np.isposinf(upper), 1.0e15, upper)
    expression = ca.vertcat(*expressions) if expressions else ca.SX.zeros(0, 1)
    if expression.numel() != lower.size or lower.size != upper.size:
        raise ValueError("Original constraint expression/bound sizes differ")
    return expression, lower, upper


def metadata_text(layer):
    settings = load_settings(layer)

    values = {
        "kN": ("int", int(settings.N)),
        "kNu": ("int", int(settings.model.nu)),
        "kNx": ("int", int(settings.model.nx)),
        "kDt": ("double", float(interval(settings))),
        "kIntegratorStepsize": (
            "double",
            float(settings.integrator_options["stepsize"]),
        ),
        "kIntegratorSteps": (
            "int",
            int(settings.integrator_options["steps"]),
        ),
    }

    stage = settings.params.params[0]
    values.update({
        "kNumberOfObjectiveParameters": ("int", stage["n_par_objectives"]),
        "kNumberOfWeightParameters": ("int", stage["n_par_weights"]),
        "kNumberOfConstraintParameters": ("int", stage["n_par_constraints"]),
        "kNumberOfAcadosParameters": (
            "int", stage["n_par"] -
            (stage["n_par_weights"] if settings.hardcode_weights else 0),
        ),
        "kNDiscs": ("int", settings.n_discs),
    })

    if layer == "tmpc":
        values["kRatio"] = ("int", int(settings.ratio))
    else:
        values["kNTotal"] = ("int", int(settings.N_total))
        values["kNStatesToConstrain"] = (
            "int",
            int(settings.N_total - settings.N + 1),
        )

    guard = f"HMPC_{layer.upper()}_SETTINGS_METADATA_H"

    lines = [
        "// Generated from original HMPC settings. Do not edit.",
        f"#ifndef {guard}",
        f"#define {guard}",
        "#include <vector>",
        "#include <string>",
        f"namespace hmpc_{layer}_settings",
        "{",
    ]

    for name, (kind, value) in values.items():
        lines.append(f"constexpr {kind} {name} = {value!r};")

    weight_names = settings.params.params[0]["par_weights_names"]

    weight_values = ", ".join(
        repr(float(settings.weights.weight_dict[name]))
        for name in weight_names
    )

    lines.extend([
        "inline std::vector<double> defaultWeights()",
        "{",
        f"  return {{{weight_values}}};",
        "}",
    ])

    parameter_names = (
        stage["par_objectives_names"] + stage["par_weights_names"] +
        stage["par_constraints_names"]
    )
    name_vectors = (
        ("stageVariableNames", settings.model.inputs + settings.model.states),
        ("objectiveParameterNames", stage["par_objectives_names"]),
        ("weightParameterNames", stage["par_weights_names"]),
        ("constraintParameterNames", stage["par_constraints_names"]),
        ("allParameterNames", parameter_names),
    )
    for name, entries in name_vectors:
        lines.extend([
            f"inline std::vector<std::string> {name}()",
            "{",
            "  return {" + ", ".join(json.dumps(entry) for entry in entries) + "};",
            "}",
        ])

    # Export the same constant metadata used by the original HMPC generator.
    # ConstantsStructure already stores matrix entries in column-major order.
    constants = settings.constants.constants
    constant_vectors = (
        ("constantNames", "std::string", [
            json.dumps(entry["name"]) for entry in constants
        ]),
        ("constantDimensions", "int", [
            str(int(entry["ndim"])) for entry in constants
        ]),
        ("constantSizes", "int", [
            str(int(size)) for entry in constants for size in entry["sizes"]
        ]),
        ("constantStartIndices", "int", [
            str(int(entry["start_idx"])) for entry in constants
        ]),
        ("constantValues", "double", [
            repr(float(value))
            for entry in constants for value in entry["values"]
        ]),
    )

    for name, kind, entries in constant_vectors:
        lines.extend([
            f"inline std::vector<{kind}> {name}()",
            "{",
            "  return {" + ", ".join(entries) + "};",
            "}",
        ])

    lines.extend(["}", f"#endif  // {guard}", ""])
    return "\n".join(lines)


def write_metadata(layer, output_directory):
    path = (
        output_directory
        / "c_generated_code"
        / f"hmpc_{layer}_settings_metadata.h"
    )
    path.write_text(metadata_text(layer))

def check_metadata(layer, output_directory):
    path = (
        output_directory
        / "c_generated_code"
        / f"hmpc_{layer}_settings_metadata.h"
    )

    if not path.is_file() or path.read_text() != metadata_text(layer):
        raise RuntimeError(
            f"{layer.upper()} settings/constants metadata is missing "
            "or stale. Regenerate its acados solver before building."
        )


def check_generated_settings():
    for layer in ("tmpc", "pmpc"):
        output_directory = (
            package_root()
            / "acados_generated"
            / f"hovergames_{layer}"
        )
        check_metadata(layer, output_directory)

    print("HMPC dimensions, timing and constants match both generated solvers.")


if __name__ == "__main__":
    check_generated_settings()