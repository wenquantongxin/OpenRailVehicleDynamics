#!/usr/bin/env python3
"""Validate and materialize the frozen INT-07A serial comparison manifest.

This module deliberately contains no runner execution, timing aggregation,
performance ranking, or default-integrator selection logic.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path, PurePosixPath
from typing import Mapping


class ManifestValidationError(ValueError):
    """The INT-07A comparison manifest violates its closed schema."""


_EXPECTED_CASES = (
    ("scenario_default_cvode_coarse", "scenario_default_cvode", "coarse", 10.0),
    ("scenario_default_cvode_nominal", "scenario_default_cvode", "nominal", 1.0),
    ("scenario_default_cvode_fine", "scenario_default_cvode", "fine", 0.1),
    (
        "scenario_default_cvode_reference",
        "scenario_default_cvode",
        "reference",
        0.01,
    ),
    ("radau5_coarse", "radau5", "coarse", 10.0),
    ("radau5_nominal", "radau5", "nominal", 1.0),
    ("radau5_fine", "radau5", "fine", 0.1),
    ("radau5_reference", "radau5", "reference", 0.01),
)

_SCENARIO_LAYOUTS = {
    "gz18_straight_aar6_v60_passive_20s": ("gz18", None),
    "irw_r300_aar5_v60_passive_30s": (
        "irw-passive-scenario",
        "irw_r300_aar5_v60_passive",
    ),
}

_FROZEN_SCENARIOS = {
    "gz18_straight_aar6_v60_passive_20s": {
        "inputs": {
            "vehicle_definition": "vehicle_library/gz18/vehicle_definition.json",
            "resolved_startup_state": (
                "vehicle_library/gz18/startup_states/moving_startup_60kmh.json"
            ),
            "track_geometry": "track_library/geometries/straight_level_1100m.json",
            "orvd_data_root": ".",
            "track_irregularity_identifier": "aar6_irregularity",
        },
        "clock": {
            "start_time_nanoseconds": 0,
            "duration_nanoseconds": 20_000_000_000,
            "sample_period_nanoseconds": 500_000,
            "external_event_period_nanoseconds": None,
        },
        "smooth_state_window": {
            "begin_time_nanoseconds": 0,
            "end_time_nanoseconds": 10_000_000,
            "require_constant_contact_patch_counts": True,
        },
        "state_layout": {
            "generalized_position_count": 57,
            "generalized_velocity_count": 50,
            "series_force_state_count": 2,
        },
        "scenario_default_numerics": {
            "integrator_recipe_identifier": "cvode_bdf2",
            "relative_tolerance": 1.0e-6,
            "generalized_position_absolute_tolerance": 1.0e-7,
            "generalized_velocity_absolute_tolerance": 1.0e-6,
            "series_force_absolute_tolerance_newtons": 1.0e-1,
        },
        "comparison_budgets": {
            "carrier_lateral_global_max_meters": 2.5e-7,
            "carrier_yaw_global_max_radians": 2.5e-7,
            "contact_force_rms_newtons": 5.0,
            "contact_force_max_newtons": 50.0,
            "endpoint_generalized_force_residual_inf_norm": 1.0e-7,
            "endpoint_absolute_virtual_power_residual_watts": 1.0e-7,
            "endpoint_position_derivative_slice_inf_norm": 1.0e-14,
            "endpoint_series_force_derivative_slice_inf_norm": 1.0e-14,
        },
    },
    "irw_r300_aar5_v60_passive_30s": {
        "inputs": {
            "vehicle_definition": "vehicle_library/irw/vehicle_definition.json",
            "resolved_startup_state": (
                "vehicle_library/irw/startup_states/moving_startup_60kmh.json"
            ),
            "track_geometry": (
                "track_library/geometries/"
                "r300_centerline_superelevation_1100m.json"
            ),
            "orvd_data_root": ".",
            "track_irregularity_identifier": "aar5_irregularity",
        },
        "clock": {
            "start_time_nanoseconds": 0,
            "duration_nanoseconds": 30_000_000_000,
            "sample_period_nanoseconds": 500_000,
            "external_event_period_nanoseconds": None,
        },
        "smooth_state_window": {
            "begin_time_nanoseconds": 0,
            "end_time_nanoseconds": 10_000_000,
            "require_constant_contact_patch_counts": True,
        },
        "state_layout": {
            "generalized_position_count": 81,
            "generalized_velocity_count": 74,
            "series_force_state_count": 2,
        },
        "scenario_default_numerics": {
            "integrator_recipe_identifier": "cvode_bdf5",
            "relative_tolerance": 1.0e-8,
            "generalized_position_absolute_tolerance": 1.0e-8,
            "generalized_velocity_absolute_tolerance": 1.0e-7,
            "series_force_absolute_tolerance_newtons": 1.0e-6,
        },
        "comparison_budgets": {
            "carrier_lateral_global_max_meters": 2.0e-6,
            "carrier_yaw_global_max_radians": 2.0e-6,
            "contact_force_rms_newtons": 25.0,
            "contact_force_max_newtons": 250.0,
            "endpoint_generalized_force_residual_inf_norm": 1.0e-7,
            "endpoint_absolute_virtual_power_residual_watts": 1.0e-7,
            "endpoint_position_derivative_slice_inf_norm": 1.0e-14,
            "endpoint_series_force_derivative_slice_inf_norm": 1.0e-14,
        },
    },
}

_RUNNER_ARGUMENT_LAYOUTS = {
    "gz18": (9, 5),
    "irw-passive-scenario": (10, 6),
}

_STATISTICS_FIELDS = (
    "successful_internal_step_count",
    "right_hand_side_evaluation_count",
    "linear_solver_right_hand_side_evaluation_count",
    "error_test_failure_count",
    "nonlinear_solver_iteration_count",
    "nonlinear_solver_convergence_failure_count",
    "linear_solver_setup_count",
    "jacobian_evaluation_count",
    "requested_dense_finite_difference_jacobian_worker_count",
)


def _reject(path: str, detail: str) -> None:
    raise ManifestValidationError(f"{path}: {detail}")


def _require_object(
    value: object, path: str, expected_keys: tuple[str, ...]
) -> dict[str, object]:
    if type(value) is not dict:
        _reject(path, "must be an object")
    result = value
    actual_keys = set(result)
    required_keys = set(expected_keys)
    if actual_keys != required_keys:
        missing = sorted(required_keys - actual_keys)
        unknown = sorted(actual_keys - required_keys)
        details: list[str] = []
        if missing:
            details.append(f"missing keys {missing!r}")
        if unknown:
            details.append(f"unknown keys {unknown!r}")
        _reject(path, "; ".join(details))
    return result


def _require_array(value: object, path: str) -> list[object]:
    if type(value) is not list:
        _reject(path, "must be an array")
    return value


def _require_string(value: object, path: str) -> str:
    if type(value) is not str or not value or value.strip() != value:
        _reject(path, "must be a non-empty string without surrounding whitespace")
    return value


def _require_bool(value: object, path: str) -> bool:
    if type(value) is not bool:
        _reject(path, "must be a boolean")
    return value


def _require_integer(value: object, path: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        _reject(path, f"must be an integer greater than or equal to {minimum}")
    return value


def _require_positive_number(value: object, path: str) -> float:
    if type(value) not in (int, float):
        _reject(path, "must be a number")
    result = float(value)
    if not math.isfinite(result) or not result > 0.0:
        _reject(path, "must be positive and finite")
    return result


def _require_exact(value: object, expected: object, path: str) -> None:
    if value != expected or type(value) is not type(expected):
        _reject(path, f"must equal {expected!r}")


def _require_string_array(
    value: object, path: str, expected: tuple[str, ...]
) -> None:
    entries = _require_array(value, path)
    actual = tuple(
        _require_string(entry, f"{path}[{index}]")
        for index, entry in enumerate(entries)
    )
    if actual != expected:
        _reject(path, f"must equal {list(expected)!r}")


def _require_relative_source_path(value: object, path: str) -> str:
    text = _require_string(value, path)
    pure_path = PurePosixPath(text)
    if pure_path.is_absolute() or ".." in pure_path.parts or "\\" in text:
        _reject(path, "must be a repository-relative POSIX path")
    if text != "." and str(pure_path) != text:
        _reject(path, "must be a normalized repository-relative POSIX path")
    return text


def _validate_required_execution(value: object) -> None:
    execution = _require_object(
        value,
        "required_execution",
        (
            "build_type",
            "floating_point_semantics_identifier",
            "jacobian_worker_count",
            "omp_num_threads",
            "omp_dynamic",
            "omp_max_active_levels",
            "omp_nested",
            "cpu_affinity_core_count",
        ),
    )
    _require_exact(execution["build_type"], "Release", "required_execution.build_type")
    _require_exact(
        execution["floating_point_semantics_identifier"],
        "orvd.strict_ieee_no_fast_math.v1",
        "required_execution.floating_point_semantics_identifier",
    )
    _require_exact(
        execution["jacobian_worker_count"],
        1,
        "required_execution.jacobian_worker_count",
    )
    _require_exact(
        execution["omp_num_threads"], 1, "required_execution.omp_num_threads"
    )
    _require_exact(
        execution["omp_dynamic"], False, "required_execution.omp_dynamic"
    )
    _require_exact(
        execution["omp_max_active_levels"],
        1,
        "required_execution.omp_max_active_levels",
    )
    _require_exact(
        execution["omp_nested"], False, "required_execution.omp_nested"
    )
    _require_exact(
        execution["cpu_affinity_core_count"],
        1,
        "required_execution.cpu_affinity_core_count",
    )


def _validate_qualification_cases(value: object) -> None:
    cases = _require_array(value, "qualification_cases")
    if len(cases) != len(_EXPECTED_CASES):
        _reject("qualification_cases", "must contain exactly eight entries")
    observed: list[tuple[str, str, str, float]] = []
    for index, value_case in enumerate(cases):
        path = f"qualification_cases[{index}]"
        case = _require_object(
            value_case, path, ("identifier", "backend", "tier", "scale")
        )
        observed.append(
            (
                _require_string(case["identifier"], f"{path}.identifier"),
                _require_string(case["backend"], f"{path}.backend"),
                _require_string(case["tier"], f"{path}.tier"),
                _require_positive_number(case["scale"], f"{path}.scale"),
            )
        )
    if tuple(observed) != _EXPECTED_CASES:
        _reject(
            "qualification_cases",
            "must be the ordered closed CVODE/Radau5 by coarse/nominal/fine/reference set",
        )


def _validate_inputs(value: object, path: str) -> None:
    inputs = _require_object(
        value,
        path,
        (
            "vehicle_definition",
            "resolved_startup_state",
            "track_geometry",
            "orvd_data_root",
            "track_irregularity_identifier",
        ),
    )
    for key in (
        "vehicle_definition",
        "resolved_startup_state",
        "track_geometry",
        "orvd_data_root",
    ):
        _require_relative_source_path(inputs[key], f"{path}.{key}")
    _require_string(
        inputs["track_irregularity_identifier"],
        f"{path}.track_irregularity_identifier",
    )


def _validate_clock(value: object, path: str) -> None:
    clock = _require_object(
        value,
        path,
        (
            "start_time_nanoseconds",
            "duration_nanoseconds",
            "sample_period_nanoseconds",
            "external_event_period_nanoseconds",
        ),
    )
    start = _require_integer(clock["start_time_nanoseconds"], f"{path}.start_time_nanoseconds")
    duration = _require_integer(
        clock["duration_nanoseconds"], f"{path}.duration_nanoseconds", minimum=1
    )
    sample_period = _require_integer(
        clock["sample_period_nanoseconds"],
        f"{path}.sample_period_nanoseconds",
        minimum=1,
    )
    if start != 0:
        _reject(f"{path}.start_time_nanoseconds", "must equal zero")
    if duration % sample_period != 0:
        _reject(path, "duration must be an integer multiple of the sample period")
    event_period = clock["external_event_period_nanoseconds"]
    if event_period is not None:
        resolved_event_period = _require_integer(
            event_period, f"{path}.external_event_period_nanoseconds", minimum=1
        )
        if duration % resolved_event_period != 0:
            _reject(path, "duration must be an integer multiple of the event period")


def _validate_state_layout(value: object, path: str) -> None:
    layout = _require_object(
        value,
        path,
        (
            "generalized_position_count",
            "generalized_velocity_count",
            "series_force_state_count",
        ),
    )
    for key in layout:
        _require_integer(layout[key], f"{path}.{key}", minimum=1)


def _validate_smooth_state_window(
    value: object, path: str, clock: Mapping[str, object]
) -> None:
    window = _require_object(
        value,
        path,
        (
            "begin_time_nanoseconds",
            "end_time_nanoseconds",
            "require_constant_contact_patch_counts",
        ),
    )
    begin = _require_integer(
        window["begin_time_nanoseconds"], f"{path}.begin_time_nanoseconds"
    )
    end = _require_integer(
        window["end_time_nanoseconds"],
        f"{path}.end_time_nanoseconds",
        minimum=1,
    )
    sample_period = clock["sample_period_nanoseconds"]
    duration = clock["duration_nanoseconds"]
    if not begin < end or end > duration:
        _reject(path, "must be a non-empty subset of the qualification clock")
    if begin % sample_period != 0 or end % sample_period != 0:
        _reject(path, "bounds must lie on the integer sample clock")
    _require_exact(
        window["require_constant_contact_patch_counts"],
        True,
        f"{path}.require_constant_contact_patch_counts",
    )


def _validate_default_numerics(value: object, path: str) -> None:
    numerics = _require_object(
        value,
        path,
        (
            "integrator_recipe_identifier",
            "relative_tolerance",
            "generalized_position_absolute_tolerance",
            "generalized_velocity_absolute_tolerance",
            "series_force_absolute_tolerance_newtons",
        ),
    )
    recipe = _require_string(
        numerics["integrator_recipe_identifier"],
        f"{path}.integrator_recipe_identifier",
    )
    if recipe not in ("cvode_bdf2", "cvode_bdf5"):
        _reject(f"{path}.integrator_recipe_identifier", "must name a CVODE BDF recipe")
    for key in (
        "relative_tolerance",
        "generalized_position_absolute_tolerance",
        "generalized_velocity_absolute_tolerance",
        "series_force_absolute_tolerance_newtons",
    ):
        _require_positive_number(numerics[key], f"{path}.{key}")


def _validate_comparison_budgets(value: object, path: str) -> None:
    budgets = _require_object(
        value,
        path,
        (
            "carrier_lateral_global_max_meters",
            "carrier_yaw_global_max_radians",
            "contact_force_rms_newtons",
            "contact_force_max_newtons",
            "endpoint_generalized_force_residual_inf_norm",
            "endpoint_absolute_virtual_power_residual_watts",
            "endpoint_position_derivative_slice_inf_norm",
            "endpoint_series_force_derivative_slice_inf_norm",
        ),
    )
    for key in budgets:
        _require_positive_number(budgets[key], f"{path}.{key}")


def _validate_scenarios(value: object) -> None:
    scenarios = _require_array(value, "scenarios")
    if len(scenarios) != len(_SCENARIO_LAYOUTS):
        _reject("scenarios", "must contain exactly the frozen GZ18 and IRW entries")
    identifiers: list[str] = []
    for index, value_scenario in enumerate(scenarios):
        path = f"scenarios[{index}]"
        scenario = _require_object(
            value_scenario,
            path,
            (
                "identifier",
                "runner_layout",
                "runner_scenario_identifier",
                "inputs",
                "clock",
                "smooth_state_window",
                "state_layout",
                "scenario_default_numerics",
                "comparison_budgets",
                "reference_pair",
            ),
        )
        identifier = _require_string(scenario["identifier"], f"{path}.identifier")
        identifiers.append(identifier)
        if identifier not in _SCENARIO_LAYOUTS:
            _reject(f"{path}.identifier", "is not an INT-07A frozen scenario")
        expected_layout, expected_runner_scenario = _SCENARIO_LAYOUTS[identifier]
        _require_exact(scenario["runner_layout"], expected_layout, f"{path}.runner_layout")
        _require_exact(
            scenario["runner_scenario_identifier"],
            expected_runner_scenario,
            f"{path}.runner_scenario_identifier",
        )
        _validate_inputs(scenario["inputs"], f"{path}.inputs")
        _validate_clock(scenario["clock"], f"{path}.clock")
        _validate_smooth_state_window(
            scenario["smooth_state_window"],
            f"{path}.smooth_state_window",
            scenario["clock"],
        )
        _validate_state_layout(scenario["state_layout"], f"{path}.state_layout")
        _validate_default_numerics(
            scenario["scenario_default_numerics"],
            f"{path}.scenario_default_numerics",
        )
        _validate_comparison_budgets(
            scenario["comparison_budgets"], f"{path}.comparison_budgets"
        )
        _require_string_array(
            scenario["reference_pair"],
            f"{path}.reference_pair",
            ("scenario_default_cvode_reference", "radau5_reference"),
        )
        frozen = _FROZEN_SCENARIOS[identifier]
        for frozen_key in (
            "inputs",
            "clock",
            "smooth_state_window",
            "state_layout",
            "scenario_default_numerics",
            "comparison_budgets",
        ):
            if scenario[frozen_key] != frozen[frozen_key]:
                _reject(
                    f"{path}.{frozen_key}",
                    "differs from the frozen INT-07A scenario identity",
                )
    if tuple(identifiers) != tuple(_SCENARIO_LAYOUTS):
        _reject("scenarios", "must retain the frozen deterministic scenario order")


def _validate_comparison_contract(value: object) -> None:
    path = "comparison_contract"
    contract = _require_object(
        value,
        path,
        (
            "time_join_columns",
            "continuous_state_artifact_filename",
            "continuous_state_layout_order",
            "continuous_state_metrics",
            "configuration_error_rule",
            "orientation_reconstruction_rule",
            "smooth_state_pairwise_metric_scope",
            "smooth_contact_patch_count_rule",
            "quaternion_norm_defect_scope",
            "normalization_scale_source",
            "candidate_values_in_normalization_denominator",
            "smooth_state_error_reduction_rule",
            "reference_band_metric_reduction_rule",
            "backend_refinement_pairing_rule",
            "reference_smooth_rms_limit",
            "reference_smooth_inf_norm_limit",
            "eligible_candidate_smooth_rms_limit",
            "eligible_candidate_smooth_inf_norm_limit",
            "reference_quaternion_norm_defect_inf_norm_limit",
            "eligible_candidate_quaternion_norm_defect_inf_norm_limit",
            "reference_band_candidate_distance_rule",
            "kinematic_entity_group",
            "physical_metric_scope",
            "carrier_error_reduction_rule",
            "contact_interface_set_rule",
            "contact_event_detection_rule",
            "contact_event_matching_rule",
            "contact_event_time_unit",
            "contact_event_tolerance_samples",
            "contact_event_guard_samples",
            "allow_unaligned_contact_pointwise_absolute_difference",
            "force_channels",
            "diagnostic_force_channels",
            "diagnostic_force_channel_rule",
            "force_bin_period_nanoseconds",
            "force_bin_quadrature_rule",
            "force_bin_outputs",
            "force_error_reduction_rule",
            "primary_timing_metric",
            "end_to_end_timing_guard_metric",
            "end_to_end_timing_guard_rule",
            "statistics_fields",
            "combine_failure_counters",
            "performance_ranking_enabled",
        ),
    )
    _require_string_array(
        contract["time_join_columns"],
        f"{path}.time_join_columns",
        ("sample_index", "time_nanoseconds"),
    )
    _require_exact(
        contract["continuous_state_artifact_filename"],
        "continuous_states.tsv",
        f"{path}.continuous_state_artifact_filename",
    )
    _require_string_array(
        contract["continuous_state_layout_order"],
        f"{path}.continuous_state_layout_order",
        ("q", "v", "z"),
    )
    _require_string_array(
        contract["continuous_state_metrics"],
        f"{path}.continuous_state_metrics",
        (
            "dimensionless_rms",
            "dimensionless_inf_norm",
            "quaternion_norm_defect_inf_norm",
        ),
    )
    _require_exact(
        contract["configuration_error_rule"],
        "free_body_rotation_log_plus_quaternion_norm_difference_plus_translation;ball_rpy_rotation_log;remaining_joint_positions_direct",
        f"{path}.configuration_error_rule",
    )
    _require_exact(
        contract["orientation_reconstruction_rule"],
        "normalize_each_finite_nonzero_quaternion_before_rotation;zero_or_nonfinite_fails",
        f"{path}.orientation_reconstruction_rule",
    )
    _require_exact(
        contract["smooth_state_pairwise_metric_scope"],
        "smooth_state_window_only;full_window_state_error_diagnostic_only",
        f"{path}.smooth_state_pairwise_metric_scope",
    )
    _require_exact(
        contract["smooth_contact_patch_count_rule"],
        "per_named_interface_positive_integer_and_constant_within_each_run_and_identical_across_reference_pair_and_eligible_candidate_trajectories",
        f"{path}.smooth_contact_patch_count_rule",
    )
    _require_exact(
        contract["quaternion_norm_defect_scope"],
        "full_continuous_state_artifact",
        f"{path}.quaternion_norm_defect_scope",
    )
    _require_exact(
        contract["normalization_scale_source"],
        "scenario_default_nominal_tolerances_and_reference_pair_magnitudes",
        f"{path}.normalization_scale_source",
    )
    _require_exact(
        contract["candidate_values_in_normalization_denominator"],
        False,
        f"{path}.candidate_values_in_normalization_denominator",
    )
    _require_exact(
        contract["smooth_state_error_reduction_rule"],
        "per_common_sample_scale_i_k_then_rms_and_global_max_within_each_q_v_z_group",
        f"{path}.smooth_state_error_reduction_rule",
    )
    _require_exact(
        contract["reference_band_metric_reduction_rule"],
        "for_state_force_and_carrier_reduce_candidate_to_each_reference_separately_then_take_larger_scalar_for_each_metric",
        f"{path}.reference_band_metric_reduction_rule",
    )
    _require_exact(
        contract["backend_refinement_pairing_rule"],
        "cvode_nominal_to_cvode_fine_to_cvode_reference;radau5_nominal_to_radau5_fine_to_radau5_reference",
        f"{path}.backend_refinement_pairing_rule",
    )
    _require_exact(
        contract["reference_smooth_rms_limit"],
        0.1,
        f"{path}.reference_smooth_rms_limit",
    )
    _require_exact(
        contract["reference_smooth_inf_norm_limit"],
        1.0,
        f"{path}.reference_smooth_inf_norm_limit",
    )
    _require_exact(
        contract["eligible_candidate_smooth_rms_limit"],
        1.0,
        f"{path}.eligible_candidate_smooth_rms_limit",
    )
    _require_exact(
        contract["eligible_candidate_smooth_inf_norm_limit"],
        10.0,
        f"{path}.eligible_candidate_smooth_inf_norm_limit",
    )
    _require_exact(
        contract["reference_quaternion_norm_defect_inf_norm_limit"],
        1.0e-6,
        f"{path}.reference_quaternion_norm_defect_inf_norm_limit",
    )
    _require_exact(
        contract["eligible_candidate_quaternion_norm_defect_inf_norm_limit"],
        1.0e-4,
        f"{path}.eligible_candidate_quaternion_norm_defect_inf_norm_limit",
    )
    _require_exact(
        contract["reference_band_candidate_distance_rule"],
        "maximum_distance_to_either_reference",
        f"{path}.reference_band_candidate_distance_rule",
    )
    _require_exact(
        contract["kinematic_entity_group"],
        "all_contact_carriers",
        f"{path}.kinematic_entity_group",
    )
    _require_exact(
        contract["physical_metric_scope"],
        "full_qualification_clock_including_both_endpoints",
        f"{path}.physical_metric_scope",
    )
    _require_exact(
        contract["carrier_error_reduction_rule"],
        "same_time_absolute_error_global_max_over_all_contact_carriers_and_full_clock",
        f"{path}.carrier_error_reduction_rule",
    )
    _require_exact(
        contract["contact_interface_set_rule"],
        "exact_identical_named_interface_set",
        f"{path}.contact_interface_set_rule",
    )
    _require_exact(
        contract["contact_event_detection_rule"],
        "sampled_patch_count_on_off;event_time_is_first_sample_of_new_state;include_t0_initial_state",
        f"{path}.contact_event_detection_rule",
    )
    _require_exact(
        contract["contact_event_matching_rule"],
        "fail_closed;initial_on_off_vector_must_match;per_interface_and_polarity_event_counts_must_match;unmatched_event_fails;match_by_interface_polarity_and_ordinal;single_off_sample_duration_is_one_tick;terminal_off_interval_is_right_censored",
        f"{path}.contact_event_matching_rule",
    )
    _require_exact(
        contract["contact_event_time_unit"],
        "integer_nanoseconds",
        f"{path}.contact_event_time_unit",
    )
    _require_exact(
        contract["contact_event_tolerance_samples"],
        1,
        f"{path}.contact_event_tolerance_samples",
    )
    _require_exact(
        contract["contact_event_guard_samples"],
        1,
        f"{path}.contact_event_guard_samples",
    )
    _require_exact(
        contract["allow_unaligned_contact_pointwise_absolute_difference"],
        False,
        f"{path}.allow_unaligned_contact_pointwise_absolute_difference",
    )
    _require_string_array(
        contract["force_channels"],
        f"{path}.force_channels",
        (
            "vertical_support_force_on_wheel_newtons",
            "normal_force_newtons",
            "total_force_on_wheel_in_carrier_track_frame_x_newtons",
            "total_force_on_wheel_in_carrier_track_frame_y_newtons",
            "total_force_on_wheel_in_carrier_track_frame_z_newtons",
        ),
    )
    _require_string_array(
        contract["diagnostic_force_channels"],
        f"{path}.diagnostic_force_channels",
        (
            "longitudinal_force_on_wheel_newtons",
            "lateral_force_on_wheel_newtons",
        ),
    )
    _require_exact(
        contract["diagnostic_force_channel_rule"],
        "maximum_normal_force_primary_patch_selector_dependent_not_eligibility_gate",
        f"{path}.diagnostic_force_channel_rule",
    )
    _require_exact(
        contract["force_bin_period_nanoseconds"],
        10_000_000,
        f"{path}.force_bin_period_nanoseconds",
    )
    _require_exact(
        contract["force_bin_quadrature_rule"],
        "composite_trapezoid_including_both_endpoints;adjacent_bins_share_boundary_sample",
        f"{path}.force_bin_quadrature_rule",
    )
    _require_string_array(
        contract["force_bin_outputs"],
        f"{path}.force_bin_outputs",
        ("impulse_equivalent_mean", "sample_minimum", "sample_maximum"),
    )
    _require_exact(
        contract["force_error_reduction_rule"],
        "rms_and_global_max_over_interface_channel_bin_output",
        f"{path}.force_error_reduction_rule",
    )
    _require_exact(
        contract["primary_timing_metric"],
        "performance_json.advance_wall_seconds",
        f"{path}.primary_timing_metric",
    )
    _require_exact(
        contract["end_to_end_timing_guard_metric"],
        "execution_identity.process_wall_seconds",
        f"{path}.end_to_end_timing_guard_metric",
    )
    _require_exact(
        contract["end_to_end_timing_guard_rule"],
        "radau5_candidate_mean_must_not_exceed_cvode_baseline_mean;report_each_arm_min_max_separately",
        f"{path}.end_to_end_timing_guard_rule",
    )
    _require_string_array(
        contract["statistics_fields"],
        f"{path}.statistics_fields",
        _STATISTICS_FIELDS,
    )
    _require_exact(
        contract["combine_failure_counters"],
        False,
        f"{path}.combine_failure_counters",
    )
    _require_exact(
        contract["performance_ranking_enabled"],
        False,
        f"{path}.performance_ranking_enabled",
    )


def validate_manifest(value: object) -> None:
    """Validate the complete closed INT-07A schema without executing a run."""

    manifest = _require_object(
        value,
        "manifest",
        (
            "schema_version",
            "manifest_identifier",
            "purpose",
            "performance_decision_eligible",
            "required_execution",
            "qualification_cases",
            "scenarios",
            "comparison_contract",
        ),
    )
    _require_exact(manifest["schema_version"], 2, "schema_version")
    _require_exact(
        manifest["manifest_identifier"],
        "int07a_serial_baseline_v2",
        "manifest_identifier",
    )
    _require_exact(
        manifest["purpose"],
        "protocol_and_serial_baseline_only",
        "purpose",
    )
    decision_eligible = _require_bool(
        manifest["performance_decision_eligible"],
        "performance_decision_eligible",
    )
    if decision_eligible:
        _reject(
            "performance_decision_eligible",
            "INT-07A serial evidence cannot authorize a performance decision",
        )
    _validate_required_execution(manifest["required_execution"])
    _validate_qualification_cases(manifest["qualification_cases"])
    _validate_scenarios(manifest["scenarios"])
    _validate_comparison_contract(manifest["comparison_contract"])


def _reject_non_finite_json_constant(value: str) -> None:
    raise ManifestValidationError(f"manifest JSON contains non-finite {value}")


def _reject_duplicate_json_object(
    pairs: list[tuple[str, object]],
) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ManifestValidationError(
                f"manifest JSON contains duplicate key {key!r}"
            )
        result[key] = value
    return result


def _validate_source_inputs(
    manifest: Mapping[str, object], source_root: os.PathLike[str] | str
) -> Path:
    try:
        root = Path(source_root).resolve(strict=True)
    except OSError as error:
        raise ManifestValidationError(
            f"source_root: could not resolve {source_root!r}: {error}"
        ) from error
    if not root.is_dir():
        _reject("source_root", "must resolve to a directory")

    for scenario_value in manifest["scenarios"]:  # type: ignore[index]
        scenario = scenario_value  # type: ignore[assignment]
        scenario_identifier = scenario["identifier"]
        inputs = scenario["inputs"]
        for key in (
            "vehicle_definition",
            "resolved_startup_state",
            "track_geometry",
            "orvd_data_root",
        ):
            relative = inputs[key]
            try:
                resolved = (root / relative).resolve(strict=True)
                resolved.relative_to(root)
            except (OSError, ValueError) as error:
                raise ManifestValidationError(
                    f"scenario {scenario_identifier!r} input {key!r} "
                    f"does not resolve inside source_root: {error}"
                ) from error
            expected_directory = key == "orvd_data_root"
            if expected_directory != resolved.is_dir():
                expected_kind = "directory" if expected_directory else "file"
                _reject(
                    f"scenario {scenario_identifier!r}.inputs.{key}",
                    f"must resolve to a {expected_kind}",
                )
    return root


def load_manifest(
    path: os.PathLike[str] | str,
    *,
    source_root: os.PathLike[str] | str | None = None,
) -> dict[str, object]:
    """Load one manifest and optionally verify its repository-relative inputs."""

    manifest_path = Path(path)
    try:
        with manifest_path.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                parse_constant=_reject_non_finite_json_constant,
                object_pairs_hook=_reject_duplicate_json_object,
            )
    except ManifestValidationError:
        raise
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestValidationError(
            f"could not load manifest {manifest_path}: {error}"
        ) from error
    validate_manifest(value)
    manifest = value
    if source_root is not None:
        _validate_source_inputs(manifest, source_root)
    return manifest


def qualification_case_identifiers(
    manifest: Mapping[str, object],
) -> tuple[str, ...]:
    """Return the validated deterministic eight-case order."""

    validate_manifest(manifest)
    return tuple(
        case["identifier"]  # type: ignore[index]
        for case in manifest["qualification_cases"]  # type: ignore[index]
    )


def materialize_runner_arguments(
    manifest: Mapping[str, object],
    *,
    scenario_identifier: str,
    qualification_case_identifier: str,
    source_root: os.PathLike[str] | str,
    output_directory: os.PathLike[str] | str,
) -> list[str]:
    """Expand one frozen scenario/case into existing runner positional argv."""

    validate_manifest(manifest)
    root = _validate_source_inputs(manifest, source_root)
    if qualification_case_identifier not in qualification_case_identifiers(manifest):
        raise KeyError(
            f"unknown INT-07A qualification case {qualification_case_identifier!r}"
        )
    scenario = next(
        (
            entry
            for entry in manifest["scenarios"]  # type: ignore[index]
            if entry["identifier"] == scenario_identifier  # type: ignore[index]
        ),
        None,
    )
    if scenario is None:
        raise KeyError(f"unknown INT-07A scenario {scenario_identifier!r}")

    output_text = os.fspath(output_directory)
    if not output_text:
        raise ValueError("output_directory must not be empty")
    output = str(Path(output_text).expanduser().resolve(strict=False))
    inputs = scenario["inputs"]  # type: ignore[index]
    vehicle = str((root / inputs["vehicle_definition"]).resolve(strict=True))
    startup = str((root / inputs["resolved_startup_state"]).resolve(strict=True))
    track = str((root / inputs["track_geometry"]).resolve(strict=True))
    data_root = str((root / inputs["orvd_data_root"]).resolve(strict=True))
    irregularity = inputs["track_irregularity_identifier"]
    clock = scenario["clock"]  # type: ignore[index]
    duration = str(clock["duration_nanoseconds"])
    sample_period = str(clock["sample_period_nanoseconds"])
    runner_layout = scenario["runner_layout"]  # type: ignore[index]

    if runner_layout == "gz18":
        arguments = [
            vehicle,
            startup,
            track,
            data_root,
            irregularity,
            output,
            duration,
            sample_period,
            qualification_case_identifier,
        ]
    elif runner_layout == "irw-passive-scenario":
        arguments = [
            scenario["runner_scenario_identifier"],  # type: ignore[index]
            vehicle,
            startup,
            track,
            data_root,
            irregularity,
            output,
            duration,
            sample_period,
            qualification_case_identifier,
        ]
    else:  # The strict validator makes this branch unreachable.
        raise AssertionError(f"unsupported runner layout {runner_layout!r}")

    expected_count, expected_output_index = _RUNNER_ARGUMENT_LAYOUTS[runner_layout]
    if len(arguments) != expected_count or arguments[expected_output_index] != output:
        raise AssertionError("materialized runner argument layout is inconsistent")
    return arguments
