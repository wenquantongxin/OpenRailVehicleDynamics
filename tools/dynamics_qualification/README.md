# Dynamics qualification tools

This directory contains private qualification executables and one generic
execution-metrics wrapper. They are not installed and do not define a public
simulation, observation, or artifact API.

General scenario-specific plotting and extraction scripts are intentionally not
maintained here. INT-07A adds the closed, machine-readable
`int07a_serial_baseline_v2` comparison manifest plus a strict validator/argv
materializer; it is a qualification
protocol surface, not a general analysis framework and does not execute or rank
runs. Temporary post-processing code may still live under the repository's
untracked `tmp/` tree while an experiment is active and may be removed after use.

## Closed qualification runners

The support library provides one private vehicle execution engine plus closed,
typed recipes. It is not an arbitrary-vehicle interface. Each recipe fixes its
scenario factory, representative bodies, state and wrench layout, integration
tolerances, track-irregularity requirement, and output schema.

The executables are:

- `orvd_gz18_dynamics_qualification`: passive GZ18 long-window runs with one
  explicitly named track-irregularity asset;
- `orvd_irw_passive_scenario`: passive IRW runs selected by a closed scenario
  identity. The current set is `irw_r300_no_irregularity_v60_passive`,
  `irw_r300_aar5_v60_passive`, `irw_straight_aar5_v80_passive`,
  `irw_r600_aar5_v80_passive`, `irw_r800_aar5_v100_passive`,
  `irw_straight_aar6_v120_passive`, `irw_r1000_aar6_v120_passive`,
  `irw_straight_aar6_v160_passive`, and
  `irw_straight_erri_low_v200_passive`;
- `orvd_irw_r300_aar5_v60_100hz_full_state_guidance`: the closed IRW
  R300/AAR5/60 km/h, 100 Hz full-state wheel-speed guidance and control-event
  scenario.

The GZ18 default recipe uses CVODE BDF2, relative tolerance `1e-6`, and
q/v/z absolute tolerances `1e-7 / 1e-6 / 0.1 N`. The R300/60 km/h
no-irregularity passive recipe and the R300/AAR5/60 km/h 100 Hz guidance recipe
also default to CVODE BDF2, with relative tolerance `1e-6` and q/v/z
absolute tolerances `1e-6 / 1e-5 / 1e-6 N`. The passive R300/AAR5/60 km/h
recipe defaults to CVODE BDF5, relative tolerance `1e-8`, and q/v/z absolute
tolerances `1e-8 / 1e-7 / 1e-6 N`. Each runner verifies the concrete backend
identity and publishes `integrator_recipe_identifier` in the numerical
execution contract. `maximum_bdf_order` remains an integer for CVODE and is
`null` for Radau5. These are private execution recipes, not a public
integrator-policy interface.

The source-tree run configurations use the closed
`TimeIntegratorQualificationCase`, which is the Cartesian product of
`scenario_default_cvode`/`radau5` and
`coarse`/`nominal`/`fine`/`reference`. All three executables accept one complete
case identifier as their optional final argument, for example
`scenario_default_cvode_fine` or `radau5_reference`. The parser recognizes only
the eight complete identifiers: there are no independent backend or tolerance
knobs, environment switch, installed API, or placeholder for Newmark/Zhai.
Omitting the argument retains each scenario's prior CVODE default and original
argument layout.
If Newmark or Zhai is implemented later, selection must evolve together with a
real concrete runtime into a tagged, method-specific configuration whose
payload owns that method's parameters and state-history policy. Adding only an
enum value, or putting unrelated method parameters into a generic option bag,
is not an admissible qualification interface.

The four tolerance scales relative to the scenario recipe are respectively
`10`, `1`, `0.1`, and `0.01`. Metadata records the complete case identifier,
tier, scale, concrete backend identity, and all resolved q/v/z tolerance
values. INT-07A freezes the representative GZ18/IRW passive serial plan in
`int07a_serial_comparison_manifest.json`; `qualification_comparison.py`
validates its exact closed schema and expands all sixteen runner/case argv
combinations without launching them. Manifest v2 also freezes
`orvd.strict_ieee_no_fast_math.v1`: CMake rejects known fast/finite-math
semantics, the sole compile launcher checks each expanded compiler command and
injects its own proof macro only after that check, and the final qualification
translation units require this proof. Qualification builds therefore require a
Ninja/Makefile generator and reject a pre-existing compiler launcher. Both
passive and controlled metadata record the actual build type, compiler and
compiled floating-point contract. The bound wrapper rejects a missing or
mismatched contract, including a compiler identity different from the binary's
self-report, after the child exits. The full state, quaternion-aware error,
dual-reference, contact-event and timing rules are frozen in the
[INT-07 equal-error protocol](../../docs/planning/integrator_migration/INT_07_EQUAL_ERROR_QUALIFICATION_PROTOCOL.md).
The v1 name denotes this floating-point safety category, not a cross-build
bit-reproduction identity. It deliberately permits compiler-default
contraction, explicit `-ffp-contract=fast/off`, and fused multiply-add
evaluation. Results from different compilers, compiler versions, targets, or
contraction choices therefore remain distinct qualification identities even
when they share the v1 category; their possible last-bit differences are
handled by the protocol's numerical budgets. Evidence remains bound to its
compiler and executable identities, while fixed-slot bitwise equality
continues to apply to different worker counts of the same binary.
Both long passive plans use a `0.5 ms` sampled-contact clock; q/v/z qualification
norms use only the preregistered `[0,10 ms]` window whose named-interface patch
counts are positive, constant within each run, and identical across the compared
trajectories, while the full trajectory remains available for non-smooth
diagnostics. Eligible candidates must also keep every normalized q/v/z group at
RMS `<=1` and maximum `<=10`; physical-response gates cannot substitute for
this state gate. Force impulse means and sampled envelopes are reduced in fixed
`10 ms` bins.
The manifest deliberately states `performance_decision_eligible=false` and
`performance_ranking_enabled=false`: the two backend-specific `reference`
cases must first agree within the preregistered budget. The default CVODE result
alone is not truth, and equal tolerance numbers do not constitute equal error.
When later stages become ranking-eligible, `advance_wall_seconds` is the primary
integrator timing; outer `process_wall_seconds` is a separately reported
end-to-end guard and may veto, but never replace, that primary metric.

The straight/AAR5 and R600/AAR5 80 km/h identities, the R800/AAR5 100 km/h
identity, the straight/AAR6 and R1000/AAR6 120 km/h identities, the
straight/AAR6 160 km/h identity, and the straight/ERRI-low 200 km/h identity
each own a separate private CVODE-BDF5 default recipe. Their relative tolerance
is `1e-9`; q/v/z absolute tolerances are `1e-9 / 1e-8 / 1e-7 N`. The settings
currently match, but the seven recipe identities remain independent. For each
higher-speed start-up, the runner requires the longitudinal speed and all eight
explicit wheel rates to equal a common scaling of the bundled 60 km/h resolved
state.

All sample times are generated from integer nanosecond identities. Intermediate
samples use dense output and do not become additional integrator stops. The
destination must not already exist; a successful run publishes one complete
directory atomically.

## Controlled IRW event semantics

The controller and torque-conditioner assets provide one common 10 ms event
period. The runner performs the frozen initialization recurrence, installs U0
before constructing the backend, and advances one zero-order-held interval at a
time. Each interval is one dense-output advance with a 0.5 ms integer mechanical
clock. Its start and end are control boundaries; the 19 intermediate observations
are not integrator stops. Adjacent boundaries are emitted once.

Dense states are replayed through a private observation context. That context
owns its own sequential carrier-projection hints and receives only the held
torque for the interval being observed. It cannot overwrite the integrator's
accepted or trial histories. At each interval end, the arriving state and contact
are observed under the preceding hold before the next periodic update is
committed.

## Published files

Passive GZ18 and IRW runners publish `continuous_states.tsv` directly from the
dense state matrix already used for observation replay. Its rows are joined by
`(sample_index,time_nanoseconds)` and its columns are the lossless `[q;v;z]`
layout recorded in metadata; `time_seconds` is audit-only. Writing it adds no
integrator stop or RHS evaluation. Raw quaternion columns are an archival
representation, not a Euclidean orientation metric; offline comparison must use
the free-quaternion and Ball-RPY rotation-log rules in the INT-07 protocol. The controlled IRW runner
does not yet publish this file because its complete long-window comparison is an
INT-08 deliverable.

`observations.tsv` keeps one fixed-width row per sample. Q and N are aggregated
over all returned patches. The historical longitudinal/lateral force convenience
columns and `primary_patch_normal_force_newtons` select the maximum-normal-force
patch. Because that selector can jump under a small perturbation, INT-07 treats
the primary-patch longitudinal/lateral columns as diagnostics; only aggregate Q,
N, and carrier Track-T total Fx/Fy/Fz are hard force channels.

`contact_patches.tsv` is the long-form contact observation: one row per sample,
named interface, and returned patch. It contains local N/Tx/Ty, the contact-frame
angle, and the compliant wheel-surface point and force expressed in the Track-T
frame at the interface projection-carrier station. Patch ordinals are local to
one evaluation and are not identities across time.

The controlled runner also writes `control_events.tsv` and
`endpoint_diagnostics.tsv`. The event table records initialization and periodic
updates. Endpoint diagnostics retain the lower-rate assembly residual and
virtual-power checks, labelled by the held-torque event used at that state.

`performance.json` reports the existing wall-clock sections and a read-only
snapshot of the configured backend's cumulative successful internal steps,
ordinary and linear-solver RHS evaluations, error-test failures, nonlinear
iterations and convergence failures, linear setups, and Jacobian evaluations.
Error-test and nonlinear-convergence failures are separate comparable
categories; neither one nor their sum is published as a generic rejected-step
count. Reading the snapshot installs no callback and performs no state or RHS
evaluation.

## Execution metrics wrapper

`run_qualification_with_metrics.py` is a Linux research wrapper for one runner
process. Select `--vehicle-recipe irw-passive-scenario` for the passive IRW
executable or
`--vehicle-recipe irw-r300-aar5-v60-100hz-full-state-guidance` for the
guidance executable. The default is the GZ18 runner.

The wrapper records outer wall time, child CPU time, peak resident memory,
executable digest, compiler/build identity, effective OpenMP environment
(including active-level and legacy nested controls), complete runner arguments,
output directory, and the CPU affinity accepted by the kernel. This is execution
provenance for a local experiment; its digest is not a physical acceptance
criterion.

For manifest-bound evidence, `--compiler-identity` must use the exact
`<compiler_id> <compiler_version>` text reported by the qualification binary;
the wrapper rejects a caller-supplied label that does not match that self-report.

Legacy wrapper calls retain that behavior. INT-07 evidence must additionally use
the all-or-none `--comparison-manifest`, `--comparison-scenario`, and
`--comparison-case` binding (with optional `--comparison-source-root`). Bound
mode rematerializes and byte-compares the complete runner argv before launch,
forces the manifest's four serial OpenMP settings, requires one applied affinity
core, and post-validates `metadata.json`, `performance.json`, `COMPLETE`, and the
three TSV artifacts. Postflight requires the exact completion count, complete
finite integer-clock state/observation rows, observation-to-patch row-count
agreement, and all nine nonnegative integration statistics. The external
identity separates `runner_exit_status` from `wrapper_exit_status`; the legacy
`exit_status` mirrors the latter, so a zero runner status cannot hide failed
postflight. It also records the manifest/scenario/case and `validation_status`;
absent or non-`passed` binding, or a floating-point compilation contract other
than the manifest's exact identity, is not INT-07 evidence.
