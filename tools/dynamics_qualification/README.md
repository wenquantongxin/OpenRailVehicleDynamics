# Dynamics qualification tools

These source-tree vehicle runners consume the public integration API. They are
not installed and do not define another simulation or solver API.

## Run a vehicle

The three executables keep their positional arguments:

```text
orvd_gz18_dynamics_qualification VEHICLE STARTUP LINE DATA_ROOT IRREGULARITY_ID OUTPUT_DIRECTORY DURATION_NS SAMPLE_PERIOD_NS [--integration-config PATH]
orvd_irw_passive_scenario SCENARIO VEHICLE STARTUP LINE DATA_ROOT IRREGULARITY_ID_OR_NONE OUTPUT_DIRECTORY DURATION_NS SAMPLE_PERIOD_NS [--integration-config PATH] [--scene-record]
orvd_irw_r300_aar5_v60_100hz_full_state_guidance VEHICLE STARTUP LINE DATA_ROOT CONTROLLER CONDITIONER OUTPUT_DIRECTORY DURATION_NS [--integration-config PATH]
```

Without `--integration-config`, a scenario uses its existing ODE defaults:

| Scenario | Method | Relative tolerance | q / v / force absolute tolerances |
|---|---|---|---|
| GZ18 passive | CVODE BDF2 | 1e-6 | 1e-7 / 1e-6 / 0.1 N |
| IRW R300, no irregularity, 60 km/h | CVODE BDF2 | 1e-6 | 1e-6 / 1e-5 / 1e-6 N |
| IRW R300 + AAR5, passive, 60 km/h | CVODE BDF5 | 1e-8 | 1e-8 / 1e-7 / 1e-6 N |
| IRW R300 + AAR5, 100 Hz guidance | CVODE BDF2 | 1e-6 | 1e-6 / 1e-5 / 1e-6 N |
| Other registered IRW passive scenarios | CVODE BDF5 | 1e-9 | 1e-9 / 1e-8 / 1e-7 N |

IRW passive scenario names are `irw_r300_no_irregularity_v60_passive`,
`irw_r300_aar5_v60_passive`, `irw_straight_aar5_v80_passive`,
`irw_r600_aar5_v80_passive`, `irw_r800_aar5_v100_passive`,
`irw_straight_aar6_v120_passive`, `irw_r1000_aar6_v120_passive`,
`irw_straight_aar6_v160_passive`, and `irw_straight_erri_low_v200_passive`.
Their vehicle, startup, line and irregularity bindings remain scenario-specific.

For example, from the repository root, with an unused output directory:

```sh
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ./build/tools/dynamics_qualification/orvd_irw_passive_scenario \
  irw_r300_aar5_v60_passive vehicle_library/irw/vehicle_definition.json \
  vehicle_library/irw/startup_states/moving_startup_60kmh.json \
  track_library/geometries/r300_centerline_superelevation_1100m.json \
  . aar5_irregularity /path/to/output 10000000 500000 \
  --integration-config /path/to/integration.json
```

## Explicit integration configuration

All runners accept the same JSON schema (`schema_version: 1`). `method` is one
of `cvode_bdf2`, `cvode_bdf5`, `radau5`, `newmark`, or `zhai`. The parser rejects
unknown fields, duplicate keys and invalid numerical values. There are no
trailing experimental cases or tolerance tiers.

ODE configurations provide `relative_tolerance`,
`generalized_position_absolute_tolerance`,
`generalized_velocity_absolute_tolerance`, and
`series_force_absolute_tolerance_newtons`. These physical-state tolerances are
expanded into the public method configuration after assembly.

Newmark and Zhai provide a positive integer `step_size_nanoseconds`, converted
to seconds once at the tool boundary. Newmark additionally provides `newton`
with `maximum_iterations` and the four scale families described in
[integration_configurations](integration_configurations/README.md). Zhai takes
no Newton settings. The library owns coordinate classification and scale
expansion; the tool never supplies private solver arrays.

`maximum_internal_steps_per_advance` is optional and defaults to the public
library limit of 1,000,000 successful internal steps. It is an execution budget,
not an accuracy setting. A valid low budget is allowed; exhaustion is reported
by the runtime. Sampling does not add integration stops.

## State and result output

Each successful run publishes `metadata.json`, `performance.json`,
`continuous_states.tsv`, `observations.tsv`, `contact_patches.tsv`, and
`COMPLETE`. Controlled runs also publish `control_events.tsv`. The IRW passive
`--scene-record` option publishes `scene_record/` from the same observation
samples; it does not change integration stops or the scene format.

Physical state `[q;v;z]` uses 17 significant digits and an integer nanosecond
sample clock. Initial and terminal samples are included, and adjacent control
intervals share one published boundary sample. Metadata describes coordinate
ownership, units and expression frames, the actual method and configuration,
input assets, and observation columns. The controlled runner retains 10 ms
holds and updates the backend only after a nonterminal control event.

`performance.json` keeps ordinary execution timings and one `integration_work`
ledger. Statistics are accumulated by increments within initialization epochs;
only successful construction or synchronization begins a new epoch. Initialization
and failed work are included when available. Construction, advance and
synchronization timings are separate from observation, control and writing.
Linear-solver setup counts retain their backend-specific meaning.

Execution metadata captures OpenMP maximum threads, dynamic-team settings,
contact worker cap and requested count once at startup. The contact cap and
request are read directly from the force plan using the same worker selection
as the contact batch. `cpu_affinity_at_start` is the startup thread's allowed
CPU set on Linux, or `null` when unavailable.
The Jacobian requested worker count remains in the integration work statistics;
zero is valid for Zhai. These describe settings, not measured CPU utilization.
The runner neither sets affinity nor rejects a serialized OpenMP team. Strict
floating-point compilation remains required.

A construction, advance or synchronization failure publishes the sibling
`OUTPUT.failure_result.json` with the stage, requested target, last public
accepted time, configuration, original exception and available work/timings.
Unavailable statistics are not filled with zeros. Results are never overwritten;
a failed run publishes neither `COMPLETE` nor a partial successful trajectory.
Failure-reporting errors do not replace the original exception.

## Boundaries

The tools assemble existing assets and call the public
`SystemContinuousStateAdvancer`. They contain no integration formulas, reference
solution selection, trajectory ranking or experiment scheduling. Observation
replay uses its own context and leaves accepted dynamics state unchanged.
