# Dynamics qualification tools

This directory contains private qualification executables and one generic
execution-metrics wrapper. They are not installed and do not define a public
simulation, observation, or artifact API.

Scenario-specific comparison and extraction scripts are intentionally not
maintained here. Temporary post-processing code may live under the repository's
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

The GZ18 recipe uses maximum BDF order 2, relative tolerance `1e-6`, and
q/v/z absolute tolerances `1e-7 / 1e-6 / 0.1 N`. The R300/60 km/h
no-irregularity passive recipe and the R300/AAR5/60 km/h 100 Hz guidance recipe
also use maximum BDF order 2, with relative tolerance `1e-6` and q/v/z
absolute tolerances `1e-6 / 1e-5 / 1e-6 N`. The passive R300/AAR5/60 km/h
recipe uses maximum BDF order 5, relative tolerance `1e-8`, and q/v/z absolute
tolerances `1e-8 / 1e-7 / 1e-6 N`. The runners verify the configured order
against the selected recipe and publish it in the numerical execution contract.
These are private execution recipes, not a public integrator-policy interface.

The straight/AAR5 and R600/AAR5 80 km/h identities, the R800/AAR5 100 km/h
identity, the straight/AAR6 and R1000/AAR6 120 km/h identities, the
straight/AAR6 160 km/h identity, and the straight/ERRI-low 200 km/h identity
each own a separate private maximum-order-5 recipe. Their relative tolerance
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

`observations.tsv` keeps one fixed-width row per sample. Q and N are aggregated
over all returned patches. The historical longitudinal/lateral force convenience
columns and `primary_patch_normal_force_newtons` select the maximum-normal-force
patch.

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
snapshot of CVODE cumulative successful internal steps, ordinary and
linear-solver RHS evaluations, error-test failures, nonlinear iterations and
convergence failures, linear setups, and Jacobian evaluations. Reading the
snapshot installs no callback and performs no state or RHS evaluation.

## Execution metrics wrapper

`run_qualification_with_metrics.py` is a Linux research wrapper for one runner
process. Select `--vehicle-recipe irw-passive-scenario` for the passive IRW
executable or
`--vehicle-recipe irw-r300-aar5-v60-100hz-full-state-guidance` for the
guidance executable. The default is the GZ18 runner.

The wrapper records outer wall time, child CPU time, peak resident memory,
executable digest, compiler/build identity, inherited OpenMP environment,
complete runner arguments, output directory, and the CPU affinity accepted by
the kernel. This is execution provenance for a local experiment; its digest is
not a physical acceptance criterion.
