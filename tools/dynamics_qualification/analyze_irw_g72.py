#!/usr/bin/env python3
"""Analyze the G72 IRW AAR5 layer and its pointwise increment over G71."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D
import numpy as np

from analyze_irw_g71 import (
    AXLES,
    BASE_COUNT,
    BASE_PERIOD_NS,
    MACRO_ALARM_MICRO,
    PLOT_LABELS,
    PLOT_SOURCE_ORDER,
    PLOT_STYLES,
    REFINEMENT_BEGIN_NS,
    REFINEMENT_END_NS,
    REFINEMENT_PERIOD_NS,
    SIMPACK_CSV_SHA256,
    STATION_GRID,
    UNION_COUNT,
    WHEELS,
    WRL_COUNT,
    WRL_CSV_SHA256,
    AnalysisError,
    contact_station_statistics,
    contact_time_statistics,
    load_json,
    plot_contact,
    plot_macro,
    plot_topology,
    read_orvd,
    read_simpack,
    read_wrl,
    require,
    sha256_file,
    statistics,
    write_json,
    zero_intervals,
)


IRREGULARITY_IDENTIFIER = "irw_r300_aar5_reference_irregularity"
SIMPACK_B_CSV_SHA256 = (
    "4c75fd5ad592ba715219909261331a49d859b6ca650dc9a772cb3755e556d0b3"
)
WRL_B_CSV_SHA256 = (
    "baa82975ea163bab2f3ac847e2f554c5eef7e39778bb2593fe44aabc094a1ce0"
)
WINDOWS = {
    "100_to_160_m_pre_activation": (100.0, 160.0),
    "160_to_200_m_fade_in": (160.0, 200.0),
    "200_to_450_m_full_excitation": (200.0, 450.0),
    "100_to_450_m_complete": (100.0, 450.0),
}


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    for layer in ("a", "b"):
        parser.add_argument(f"--orvd-{layer}-artifact", type=Path,
                            required=True)
        parser.add_argument(f"--orvd-{layer}-execution-identity", type=Path,
                            required=True)
        parser.add_argument(f"--simpack-{layer}-realtime-csv", type=Path,
                            required=True)
        parser.add_argument(f"--simpack-{layer}-contact-npz", type=Path,
                            required=True)
        parser.add_argument(f"--simpack-{layer}-run-manifest", type=Path,
                            required=True)
        parser.add_argument(f"--wrl-{layer}-csv", type=Path, required=True)
        parser.add_argument(f"--wrl-{layer}-result-json", type=Path,
                            required=True)
        parser.add_argument(f"--wrl-{layer}-run-manifest", type=Path,
                            required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args(list(argv))


def load_execution(path: Path, artifact: Path, layer: str) -> dict[str, Any]:
    value = load_json(path, f"ORVD layer-{layer} execution identity")
    require(value.get("exit_status") == 0 and
            value.get("vehicle_recipe") == "irw" and
            value.get("build_type") == "Release" and
            Path(str(value.get("qualification_artifact_directory"))).resolve() ==
            artifact.resolve(),
            f"ORVD layer-{layer} execution identity does not name its artifact")
    return value


def verify_paired_orvd_executions(a: dict[str, Any], b: dict[str, Any]) -> None:
    for key in ("orvd_revision", "build_type", "compiler", "hardware",
                "applied_cpu_affinity", "openmp_environment",
                "executable_sha256"):
        require(a.get(key) == b.get(key),
                f"ORVD A/B execution identity differs at {key}")
    a_args = a.get("runner_arguments")
    b_args = b.get("runner_arguments")
    require(isinstance(a_args, list) and isinstance(b_args, list) and
            len(a_args) == len(b_args) == 11,
            "ORVD A/B runner arguments do not have the G72 closed shape")
    require(a_args[:4] == b_args[:4] and a_args[6:] == b_args[6:] and
            a_args[4] == "none" and b_args[4] == IRREGULARITY_IDENTIFIER,
            "ORVD A/B runs differ by more than irregularity identity and output")


def base_view(source: dict[str, Any], row_count: int,
              selector: np.ndarray) -> dict[str, Any]:
    return {
        key: value[selector]
        for key, value in source.items()
        if isinstance(value, np.ndarray) and value.shape[:1] == (row_count,)
    }


def macro_station_values(
    layers: dict[str, dict[str, dict[str, Any]]]
) -> dict[str, dict[str, dict[str, np.ndarray]]]:
    output: dict[str, dict[str, dict[str, np.ndarray]]] = {}
    for layer, sources in layers.items():
        output[layer] = {source: {} for source in sources}
        for source_name, source in sources.items():
            for axle_index, axle in enumerate(AXLES):
                station = source["station"][:, axle_index]
                require(bool(np.all(np.diff(station) > 0.0)) and
                        station[0] <= STATION_GRID[0] and
                        station[-1] >= STATION_GRID[-1],
                        f"{source_name} layer {layer} {axle} lacks the grid")
                for quantity in ("lateral", "yaw"):
                    output[layer][source_name][f"{axle}.{quantity}"] = np.interp(
                        STATION_GRID, station, source[quantity][:, axle_index]
                    )
    return output


def macro_statistics(
    values: dict[str, dict[str, dict[str, np.ndarray]]]
) -> tuple[dict[str, Any], bool]:
    output: dict[str, Any] = {}
    qualified = True
    for axle in AXLES:
        output[axle] = {}
        for quantity in ("lateral", "yaw"):
            key = f"{axle}.{quantity}"
            e0 = values["a"]["orvd"][key] - values["a"]["simpack"][key]
            b_direct = (values["b"]["orvd"][key] -
                        values["b"]["simpack"][key])
            increments = {
                source: values["b"][source][key] - values["a"][source][key]
                for source in ("simpack", "wrl", "orvd")
            }
            delta_eir = increments["orvd"] - increments["simpack"]
            delta_wrl = increments["wrl"] - increments["simpack"]
            output[axle][quantity] = {}
            for window_name, (begin, end) in WINDOWS.items():
                selected = (STATION_GRID >= begin) & (STATION_GRID <= end)
                result = {
                    "e0_orvd_A_minus_simpack_A_micro": statistics(
                        e0[selected], 1.0e6, STATION_GRID[selected]),
                    "orvd_B_minus_simpack_B_micro": statistics(
                        b_direct[selected], 1.0e6, STATION_GRID[selected]),
                    "delta_eir_micro": statistics(
                        delta_eir[selected], 1.0e6, STATION_GRID[selected]),
                    "wrl_increment_minus_simpack_increment_micro": statistics(
                        delta_wrl[selected], 1.0e6, STATION_GRID[selected]),
                }
                output[axle][quantity][window_name] = result
                if window_name == "100_to_450_m_complete":
                    for identity in ("orvd_B_minus_simpack_B_micro",
                                     "delta_eir_micro"):
                        qualified &= (
                            result[identity]["rms"] <= MACRO_ALARM_MICRO and
                            result[identity]["maximum_absolute"] <=
                            MACRO_ALARM_MICRO
                        )
    return output, qualified


def topology_summary(time: np.ndarray, source: dict[str, Any]) -> dict[str, Any]:
    wheels_in_contact = np.sum(source["patch_count"] > 0, axis=1)
    return {
        "minimum_wheels_in_contact": int(np.min(wheels_in_contact)),
        "samples_with_fewer_than_eight_wheels": int(np.count_nonzero(
            wheels_in_contact < 8)),
        "peak_patch_count": int(np.max(source["patch_count"])),
        "zero_patch_intervals": zero_intervals(time, source["patch_count"]),
    }


def endpoint_diagnostics(metadata: dict[str, Any], layer: str) -> dict[str, float]:
    value = metadata.get("endpoint_assembly_and_state_slice_diagnostics")
    required = (
        "generalized_force_residual_inf_norm",
        "virtual_power_residual_watts",
        "position_derivative_slice_consistency_inf_norm",
        "series_force_derivative_slice_consistency_inf_norm",
    )
    require(isinstance(value, dict) and all(
        isinstance(value.get(key), (int, float)) and
        np.isfinite(float(value[key])) for key in required
    ), f"ORVD layer {layer} lacks finite endpoint diagnostics")
    return {key: float(value[key]) for key in required}


def plot_readme_force_summary(
    path: Path,
    base_time: np.ndarray,
    time_values: dict[str, dict[str, np.ndarray]],
    station_values: dict[str, dict[str, np.ndarray]],
    time_statistics: dict[str, Any],
) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(16, 11))
    wheel_labels = {
        "ff_l": "FF left", "ff_r": "FF right",
        "fr_l": "FR left", "fr_r": "FR right",
        "rf_l": "RF left", "rf_r": "RF right",
        "rr_l": "RR left", "rr_r": "RR right",
    }
    for row, quantity in enumerate(("Q", "Tx", "Ty")):
        wheel = max(
            WHEELS,
            key=lambda name: time_statistics[name][quantity][
                "orvd_minus_simpack"]["maximum_absolute"],
        )
        wheel_index = WHEELS.index(wheel)
        source_order = (("orvd", "simpack") if quantity == "Q" else
                        PLOT_SOURCE_ORDER)
        for column, (x, values, domain) in enumerate((
            (base_time, time_values, "time sequence"),
            (STATION_GRID, station_values, "same station"),
        )):
            axis = axes[row, column]
            for source in source_order:
                axis.plot(x, values[quantity][source][:, wheel_index] / 1000.0,
                          label=PLOT_LABELS[source],
                          **PLOT_STYLES[source])
            axis.set_title(
                f"{quantity}: {wheel_labels[wheel]} — {domain}")
            axis.set_ylabel(f"Wheel-side {quantity} [kN]")
            axis.set_xlabel("Time [s]" if column == 0 else
                            "Own axle-bridge station [m]")
            axis.grid(True, alpha=0.25)
    handles = [
        Line2D([], [], label=PLOT_LABELS[source], **PLOT_STYLES[source])
        for source in PLOT_SOURCE_ORDER
    ]
    figure.suptitle(
        "G72 IRW R300+AAR5 — canonical wheel-side force response",
        y=0.985)
    figure.text(
        0.5, 0.952,
        "Each row shows the wheel with the largest native-time "
        "ORVD–SIMPACK peak for that component",
        ha="center", va="top", fontsize=10)
    figure.legend(handles, [PLOT_LABELS[source] for source in PLOT_SOURCE_ORDER],
                  loc="upper center", ncol=3, bbox_to_anchor=(0.5, 0.932),
                  frameon=False)
    figure.subplots_adjust(left=0.075, right=0.99, bottom=0.065, top=0.88,
                           hspace=0.36, wspace=0.22)
    figure.savefig(path, dpi=220)
    plt.close(figure)


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    output = arguments.output_directory.resolve()
    if output.exists():
        print(f"G72 analysis failed: output already exists: {output}",
              file=sys.stderr)
        return 1
    try:
        orvd_a_path = arguments.orvd_a_artifact.resolve()
        orvd_b_path = arguments.orvd_b_artifact.resolve()
        orvd_raw_a = read_orvd(orvd_a_path, None, "A")
        orvd_raw_b = read_orvd(orvd_b_path, IRREGULARITY_IDENTIFIER, "B")
        execution_a = load_execution(
            arguments.orvd_a_execution_identity.resolve(), orvd_a_path, "A")
        execution_b = load_execution(
            arguments.orvd_b_execution_identity.resolve(), orvd_b_path, "B")
        verify_paired_orvd_executions(execution_a, execution_b)

        simpack_a = read_simpack(
            arguments.simpack_a_realtime_csv.resolve(),
            arguments.simpack_a_contact_npz.resolve(), SIMPACK_CSV_SHA256, "A")
        simpack_b = read_simpack(
            arguments.simpack_b_realtime_csv.resolve(),
            arguments.simpack_b_contact_npz.resolve(), SIMPACK_B_CSV_SHA256, "B")
        wrl_a = read_wrl(
            arguments.wrl_a_csv.resolve(), arguments.wrl_a_result_json.resolve(),
            arguments.wrl_a_run_manifest.resolve(), WRL_CSV_SHA256,
            "A_passive_smooth", "A")
        wrl_b = read_wrl(
            arguments.wrl_b_csv.resolve(), arguments.wrl_b_result_json.resolve(),
            arguments.wrl_b_run_manifest.resolve(), WRL_B_CSV_SHA256,
            "B_passive_aar5", "B")

        simpack_manifests = {
            layer: load_json(path.resolve(), f"SIMPACK layer-{layer} manifest")
            for layer, path in (
                ("a", arguments.simpack_a_run_manifest),
                ("b", arguments.simpack_b_run_manifest),
            )
        }
        require(simpack_manifests["a"].get("completed") is True and
                simpack_manifests["a"].get("duration_s") == 30.0 and
                simpack_manifests["a"].get("arm") == "A_passive_smooth_R300",
                "SIMPACK A manifest is not the paired frozen layer")
        require(simpack_manifests["b"].get("completed") is True and
                simpack_manifests["b"].get("duration_s") == 30.0 and
                simpack_manifests["b"].get("arm") == "B" and
                simpack_manifests["b"].get("role") ==
                "passive_P179_AAR5_R300",
                "SIMPACK B manifest is not the paired frozen layer")
        for layer, manifest, source in (
            ("A", simpack_manifests["a"], simpack_a),
            ("B", simpack_manifests["b"], simpack_b),
        ):
            artifacts = manifest.get("artifacts_sha256")
            require(isinstance(artifacts, dict) and
                    artifacts.get("realtime.csv") ==
                    source["realtime_sha256"] and
                    artifacts.get("P179_H3_TYPE109_RT.sbr") ==
                    source["sbr_sha256"],
                    f"SIMPACK {layer} realtime and contact sources do not "
                    "belong to the same frozen run")

        layers = {
            "a": {"orvd": orvd_raw_a, "simpack": simpack_a, "wrl": wrl_a},
            "b": {"orvd": orvd_raw_b, "simpack": simpack_b, "wrl": wrl_b},
        }
        station_values = macro_station_values(layers)
        macro_stats, qualified = macro_statistics(station_values)
        increments = {
            source: {
                key: station_values["b"][source][key] -
                     station_values["a"][source][key]
                for key in station_values["b"][source]
            }
            for source in ("simpack", "wrl", "orvd")
        }

        base_selector = orvd_raw_b["base_mask"]
        orvd_b = base_view(orvd_raw_b, UNION_COUNT, base_selector)
        wrl_base_indices = np.arange(0, WRL_COUNT, 5, dtype=np.int64)
        wrl_b_base = base_view(wrl_b, WRL_COUNT, wrl_base_indices)
        base_time = np.arange(BASE_COUNT, dtype=np.float64) * 0.0005
        require(np.max(np.abs(orvd_b["time"] - base_time)) <= 4.0e-15 and
                np.max(np.abs(simpack_b["time"] - base_time)) <= 4.0e-15 and
                np.max(np.abs(wrl_b_base["time"] - base_time)) <= 4.0e-15,
                "G72 base clocks are not the same integer sequence")
        macro_time_sources = {
            "orvd": {key: orvd_b[key] for key in
                     ("station", "lateral", "yaw")},
            "simpack": {key: simpack_b[key] for key in
                        ("station", "lateral", "yaw")},
            "wrl": {key: wrl_b_base[key] for key in
                    ("station", "lateral", "yaw")},
        }
        macro_time_values = {
            source: {
                f"{axle}.{quantity}":
                    macro_time_sources[source][quantity][:, axle_index]
                for axle_index, axle in enumerate(AXLES)
                for quantity in ("lateral", "yaw")
            }
            for source in ("simpack", "wrl", "orvd")
        }
        macro_time_stats = {
            axle: {
                quantity: {
                    label: statistics(
                        macro_time_sources[left][quantity][:, axle_index] -
                        macro_time_sources[right][quantity][:, axle_index],
                        1.0e6, base_time)
                    for label, left, right in (
                        ("orvd_minus_simpack", "orvd", "simpack"),
                        ("wrl_minus_simpack", "wrl", "simpack"),
                        ("orvd_minus_wrl", "orvd", "wrl"),
                    )
                }
                for quantity in ("lateral", "yaw")
            }
            for axle_index, axle in enumerate(AXLES)
        }
        same_time_station_stats = {
            axle: {
                label: statistics(
                    macro_time_sources[left]["station"][:, axle_index] -
                    macro_time_sources[right]["station"][:, axle_index],
                    coordinate=base_time)
                for label, left, right in (
                    ("orvd_minus_simpack", "orvd", "simpack"),
                    ("wrl_minus_simpack", "wrl", "simpack"),
                    ("orvd_minus_wrl", "orvd", "wrl"),
                )
            }
            for axle_index, axle in enumerate(AXLES)
        }

        contact_time_sources = {
            "orvd": {key: orvd_b[key] for key in
                     ("Q", "N_total", "N", "Tx", "Ty", "patch_count")},
            "simpack": {key: simpack_b[key] for key in
                        ("Q", "N_total", "N", "Tx", "Ty", "patch_count")},
            "wrl": {key: wrl_b_base[key] for key in
                    ("N", "Tx", "Ty", "patch_count")},
        }
        for source, station_source in (("orvd", orvd_b),
                                       ("simpack", simpack_b),
                                       ("wrl", wrl_b_base)):
            contact_time_sources[source]["station"] = station_source["station"]
        force_time_stats, force_time_values = contact_time_statistics(
            contact_time_sources, base_time)
        contact_station_sources = {
            "orvd": {key: orvd_raw_b[key] for key in
                     ("Q", "N_total", "N", "Tx", "Ty", "patch_count", "station")},
            "simpack": {key: simpack_b[key] for key in
                        ("Q", "N_total", "N", "Tx", "Ty", "patch_count", "station")},
            "wrl": {key: wrl_b[key] for key in
                    ("N", "Tx", "Ty", "patch_count", "station")},
        }
        force_station_stats, force_station_values = contact_station_statistics(
            contact_station_sources)

        fine_selector = orvd_raw_b["fine_mask"]
        orvd_fine = base_view(orvd_raw_b, UNION_COUNT, fine_selector)
        fine_begin = REFINEMENT_BEGIN_NS // REFINEMENT_PERIOD_NS
        fine_end = REFINEMENT_END_NS // REFINEMENT_PERIOD_NS
        wrl_fine = base_view(
            wrl_b, WRL_COUNT, np.arange(fine_begin, fine_end + 1))
        require(np.max(np.abs(orvd_fine["time"] - wrl_fine["time"])) <=
                4.0e-15, "G72 local ORVD/WRL clocks differ")

        output.mkdir(parents=True)
        plot_macro(output / "irw_g72_B_macro_same_station.png", STATION_GRID,
                   station_values["b"], "Own axle-bridge station [m]",
                   "IRW layer B AAR5: same-station response over 100–450 m")
        plot_macro(output / "irw_g72_AAR5_increment_same_station.png",
                   STATION_GRID, increments, "Own axle-bridge station [m]",
                   "IRW AAR5 increment B−A: same-station response")
        plot_macro(output / "irw_g72_B_macro_same_time.png", base_time,
                   macro_time_values, "Time [s]",
                   "IRW layer B AAR5: native 0.5 ms same-time response")
        for quantity in ("Q", "N", "Tx", "Ty"):
            source_order = (("orvd", "simpack") if quantity == "Q" else
                            PLOT_SOURCE_ORDER)
            plot_contact(
                output / f"irw_g72_contact_{quantity}_same_time.png",
                base_time, force_time_values[quantity], quantity, source_order,
                "Time [s]", "native 0.5 ms same time", "B")
            plot_contact(
                output / f"irw_g72_contact_{quantity}_same_station.png",
                STATION_GRID, force_station_values[quantity], quantity,
                source_order, "Own axle-bridge station [m]",
                "same station over 100–450 m", "B")
        plot_readme_force_summary(
            output / "irw_g72_wheel_force_response.png", base_time,
            force_time_values, force_station_values, force_time_stats)
        topology_base = {
            name: {"time": base_time,
                   "patch_count": contact_time_sources[name]["patch_count"]}
            for name in ("simpack", "wrl", "orvd")
        }
        plot_topology(output / "irw_g72_topology_full.png", 0.0, 30.0,
                      topology_base,
                      "IRW layer B: full-window contact topology (observation)")
        plot_topology(
            output / "irw_g72_topology_3p64_3p68s.png", 3.64, 3.68,
            {
                "simpack": {"time": simpack_b["time"],
                            "patch_count": simpack_b["patch_count"]},
                "wrl": {"time": wrl_fine["time"],
                        "patch_count": wrl_fine["patch_count"]},
                "orvd": {"time": orvd_fine["time"],
                         "patch_count": orvd_fine["patch_count"]},
            },
            "IRW layer B: contact topology, 3.64–3.68 s "
            "(ORVD/WRL 100 us; SIMPACK native 0.5 ms)")

        performance = {}
        for layer, raw, execution in (("a", orvd_raw_a, execution_a),
                                      ("b", orvd_raw_b, execution_b)):
            wall = execution.get("process_wall_seconds")
            peak = execution.get("maximum_resident_set_kilobytes")
            require(isinstance(wall, (int, float)) and wall > 0 and
                    isinstance(peak, (int, float)) and peak > 0,
                    f"ORVD layer {layer} lacks performance identity")
            performance[layer] = {
                "process_wall_seconds": float(wall),
                "simulated_seconds_per_wall_second": 30.0 / float(wall),
                "wall_seconds_per_simulated_second": float(wall) / 30.0,
                "peak_resident_memory_mib": float(peak) / 1024.0,
                "runner_performance": raw["performance"],
            }
        performance["B_over_A_wall_time_ratio"] = (
            performance["b"]["process_wall_seconds"] /
            performance["a"]["process_wall_seconds"]
        )

        payload = {
            "classification": (
                "irw_B_layer_R300_AAR5_30s_macro_and_increment_qualified"
                if qualified else
                "irw_B_layer_R300_AAR5_30s_macro_or_increment_not_qualified"),
            "qualified": qualified,
            "scope": "IRW passive R300 with frozen P179 AAR5, 30 s",
            "contract": {
                "same_station_grid_meters": [100.0, 450.0, 0.01],
                "windows_meters": WINDOWS,
                "macro_alarm_micro": MACRO_ALARM_MICRO,
                "pointwise_increment_formula": (
                    "delta_eir=(ORVD_B-ORVD_A)-(SIMPACK_B-SIMPACK_A); "
                    "statistics are computed only after pointwise subtraction"),
                "track_irregularity_identifier": IRREGULARITY_IDENTIFIER,
                "base_clock": {"period_nanoseconds": BASE_PERIOD_NS,
                               "sample_count": BASE_COUNT},
                "local_topology_clock": {
                    "begin_nanoseconds": REFINEMENT_BEGIN_NS,
                    "end_nanoseconds": REFINEMENT_END_NS,
                    "period_nanoseconds": REFINEMENT_PERIOD_NS,
                    "sample_count": 401,
                },
                "union_sample_count": UNION_COUNT,
                "no_control_or_energy_qualification": True,
                "contact_force_acceptance_threshold": None,
                "contact_topology_is_observation_not_safety_qualification": True,
            },
            "same_station_macro_statistics": macro_stats,
            "same_time_B_macro_error_micro": macro_time_stats,
            "same_time_B_station_difference_meters": same_time_station_stats,
            "endpoint_assembly_and_state_slice_diagnostics": {
                "A": endpoint_diagnostics(orvd_raw_a["metadata"], "A"),
                "B": endpoint_diagnostics(orvd_raw_b["metadata"], "B"),
            },
            "same_time_contact_force_error_newtons": force_time_stats,
            "same_station_contact_force_error_newtons": force_station_stats,
            "contact_topology_observation": {
                "layer_A_base_clock": {
                    "simpack": topology_summary(simpack_a["time"], simpack_a),
                    "wrl": topology_summary(
                        wrl_a["time"][::5],
                        base_view(wrl_a, WRL_COUNT,
                                  np.arange(0, WRL_COUNT, 5, dtype=np.int64))),
                    "orvd": topology_summary(
                        orvd_raw_a["time"][orvd_raw_a["base_mask"]],
                        base_view(orvd_raw_a, UNION_COUNT,
                                  orvd_raw_a["base_mask"])),
                },
                "layer_B_base_clock": {
                    source: topology_summary(base_time,
                                             contact_time_sources[source])
                    for source in ("simpack", "wrl", "orvd")
                },
                "layer_B_local_100us_clock": {
                    "wrl": zero_intervals(wrl_fine["time"],
                                          wrl_fine["patch_count"]),
                    "orvd": zero_intervals(orvd_fine["time"],
                                           orvd_fine["patch_count"]),
                },
            },
            "performance": performance,
            "source_identity": {
                "orvd_revision": execution_b.get("orvd_revision"),
                "orvd_executable_sha256": execution_b.get("executable_sha256"),
                "orvd_A_observations_sha256": orvd_raw_a["observation_sha256"],
                "orvd_B_observations_sha256": orvd_raw_b["observation_sha256"],
                "simpack_A_realtime_sha256": simpack_a["realtime_sha256"],
                "simpack_B_realtime_sha256": simpack_b["realtime_sha256"],
                "simpack_A_sbr_sha256": simpack_a["sbr_sha256"],
                "simpack_B_sbr_sha256": simpack_b["sbr_sha256"],
                "wrl_A_csv_sha256": wrl_a["csv_sha256"],
                "wrl_B_csv_sha256": wrl_b["csv_sha256"],
                "simpack_A_contact_npz_sha256": sha256_file(
                    arguments.simpack_a_contact_npz.resolve()),
                "simpack_B_contact_npz_sha256": sha256_file(
                    arguments.simpack_b_contact_npz.resolve()),
            },
            "figures": sorted(path.name for path in output.glob("*.png")),
        }
        write_json(output / "irw_g72_analysis.json", payload)
    except (AnalysisError, KeyError, OSError, ValueError) as error:
        print(f"G72 analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"wrote G72 analysis to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
