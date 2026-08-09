# Dynamics qualification tools

This directory contains migration-only executables and analysis scripts. They
are neither installed nor a public simulation/output API. Long-run trajectories,
reference arrays, figures, performance logs, and comparison statistics stay
outside the repository.

## GZ18 long-window runner

`orvd_gz18_dynamics_qualification` assembles one private GZ18 scenario and
advances it once to the requested terminal time. Integer nanosecond sample
identities select dense-output observations; they do not become integrator stop
times. The destination must not already exist, and a successful run publishes
one complete directory atomically.

The G60 science run uses:

```text
duration_nanoseconds = 10000000000
sample_period_nanoseconds = 500000
track_irregularity_identifier = gz18_aar6_reference_irregularity
vehicle_layout_reference_track_station_meters = 0
```

`run_qualification_with_metrics.py` is a Linux research wrapper for one such
process. It pins the requested logical processors and records the outer wall
time, child CPU time, peak resident memory, executable digest, compiler/build
identity, inherited OpenMP environment, complete runner arguments, output
directory, and the CPU affinity actually accepted by the kernel. The analysis
rejects an execution record belonging to another artifact. This is external
execution provenance; its digest is not a physical acceptance criterion.

## G60 comparison

`analyze_gz18_g60.py` compares one complete ORVD artifact against explicit
P039 paired arrays and the P038 native SIMPACK contact archive. It does not
search for either reference. NumPy and Matplotlib are research-tool
dependencies only; they are not ORVD package dependencies.

The analysis:

- verifies the 20,001-sample integer clock and named wheelset/wheel mappings;
- converts P039's declared common `W` macro response to the ORVD standard
  track `T` frame with the fixed transform `diag(1,-1,-1)`: lateral
  displacement and yaw are negated exactly once; this is a source-coordinate
  conversion, not a result-dependent sign fit;
- recomputes each wheelset's same-time `[100, 150] m` mask from SIMPACK and the
  current ORVD stations (the historical WRL masks in P039 are not reused);
- reports raw and own-`t=0`-increment errors in time and station domains;
- treats P039 `Tx/Ty` as already converted to the canonical wheel end;
- keeps `Q` (vertical support) distinct from local normal force `N`;
- generates one four-wheelset response figure and separate Q/N/Tx/Ty figures;
- reports contact topology without turning transient patch changes into a
  pass/fail rule.

The required execution-identity JSON has schema revision 1 and these fields:

```json
{
  "schema_version": 1,
  "orvd_revision": "full Git object name",
  "build_type": "Release",
  "compiler": "compiler identity",
  "hardware": "processor identity",
  "requested_cpu_affinity": "0-7",
  "applied_cpu_affinity": [0, 1, 2, 3, 4, 5, 6, 7],
  "openmp_environment": {
    "OMP_NUM_THREADS": "8",
    "OMP_DYNAMIC": "FALSE",
    "OMP_PLACES": "cores",
    "OMP_PROC_BIND": "close"
  },
  "runner_arguments": [
    "vehicle definition", "startup state", "track geometry", "data root",
    "irregularity identifier", "output directory", "duration ns", "period ns"
  ],
  "qualification_artifact_directory": "/absolute/output/directory",
  "process_wall_seconds": 123.45,
  "maximum_resident_set_kilobytes": 65536,
  "executable_sha256": "sha256",
  "exit_status": 0
}
```

The two measured values must be positive in a real record. The analysis output
directory must not exist. Use `--include-historical-wrl` only with an explicit
`--historical-wrl-source-revision`; that line is a named historical control,
not a current WRL run.
