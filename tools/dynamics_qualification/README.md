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
- `orvd_irw_dynamics_qualification`: passive IRW runs, with either no track
  irregularity or the explicitly named R300 AAR5 reference field;
- `orvd_irw_p179_controlled_qualification`: the closed IRW
  H3/R300/AAR5/100 Hz controlled scenario.

The GZ18 recipe uses relative tolerance `1e-6` and q/v/z absolute tolerances
`1e-7 / 1e-6 / 0.1 N`. The two passive IRW recipes and the controlled IRW
runner use relative tolerance `1e-7` and q/v/z absolute tolerances
`1e-9 / 1e-8 / 1e-6 N`. These are private execution recipes, not a public
integrator-policy interface.

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
process. Select `--vehicle-recipe irw` for the passive IRW executable or
`--vehicle-recipe irw-p179-controlled` for the controlled executable. The
default is the GZ18 runner.

The wrapper records outer wall time, child CPU time, peak resident memory,
executable digest, compiler/build identity, inherited OpenMP environment,
complete runner arguments, output directory, and the CPU affinity accepted by
the kernel. This is execution provenance for a local experiment; its digest is
not a physical acceptance criterion.
