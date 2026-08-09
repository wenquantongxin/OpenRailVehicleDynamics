# Dynamics qualification tools

This directory contains migration-only executables and analysis scripts. They
are neither installed nor a public simulation/output API. Long-run trajectories,
reference arrays, figures, performance logs, and comparison statistics stay
outside version control. They may be retained under the repository's untracked
`tmp/` tree.

## GZ18 long-window runner

`orvd_gz18_dynamics_qualification` assembles one private GZ18 scenario and
advances it once to the requested terminal time. Integer nanosecond sample
identities select dense-output observations; they do not become integrator stop
times. The destination must not already exist, and a successful run publishes
one complete directory atomically.

The G60 and G61 science runs use the same physical and numerical identity; only
the terminal duration changes:

```text
G60 duration_nanoseconds = 10000000000
G61 duration_nanoseconds = 20000000000
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

The required execution-identity JSON has one current field set. It carries no
numeric format revision and is not dispatched through historical formats:

```json
{
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

## G61 comparison

`analyze_gz18_g61.py` accepts only the independent P057 20 s paired arrays and
native SIMPACK contact core. It rejects P039/P038 response inputs rather than
trying to infer which Goal a file belongs to. The native core cross-checks that
Q, N, and patch count are unchanged and that SIMPACK Type-80 Tx/Ty were
converted from the rail end to the wheel end exactly once.

For each wheelset, the primary comparison independently interpolates ORVD and
SIMPACK by their own strictly increasing station onto the common 0.01 m grid.
It reports pre-activation, 50--100 m fade-in, 100--250 m full excitation,
250--300 m fade-out, post-300 m recovery, and the aggregate 50--300 m window.
The common support must reach at least 324 m. Same-time curves remain a phase
and propagation diagnostic; they do not replace the same-station gate.

The P057 macro arrays use their declared common W frame, so lateral displacement
and yaw are negated once to enter the ORVD standard track T frame. Q/N/Tx/Ty
are already canonical wheel-side scalars and are not transformed again. The
optional `drake_*` columns are historical WRL results and require the exact
historical source revision in the command line and figure legend.

`gz18_qualification_analysis_common.py` is private tool code. It shares only
artifact parsing, fixed naming, execution identity, and numerical statistics;
the G60 and G61 reference schemas and acceptance windows remain separate.

## G62 curve-station comparison

`analyze_gz18_g62.py` qualifies the R300 geometry and two distinct station
observers before any curve-response claim is made. The SIMPACK primary observer
is the mean of the left/right rail-profile reference marker stations; the WRL
control observer is the projected wheelset rigid-body origin. They are never
cross-compared under one label. The tool reports native-time station error and
native first-sample arrival time on a station grid, without shifting either
trajectory. It also validates that the packaged P040 Track-T AAR5 JSON arrays
round-trip every binary64 station and value.

## G63 curve-response comparison

`extract_gz18_g63_p047.py` reads one explicitly named P047 binary64 SBR and
publishes a compact, analysis-only archive. It requires the qualified source
digest, 5,162 channels, the exact 32,001-sample clock, and all 64 selected
channel identities. SIMPACK Type-80 `Tx/Ty` are rail-end quantities and are
negated exactly once into the canonical wheel-end convention; `Q` remains
distinct from local normal force `N`.

`analyze_gz18_g63.py` accepts only explicit ORVD, P047, P040, current-WRL and
historical-WRL inputs. P040 is a binary32 cross-check: down-converted P047
`Q/N/Tx/Ty` and patch counts must reproduce it exactly; it is not used to fill
missing P047 channels. The current WRL run is the recent control, while P055 is
named as a historical control and is compared only on its native 32,000-sample
intersection.

The macro comparison reconstructs the same excited Track-T coordinates used by
the reference: SIMPACK consumes its joint and moving-track channels, while ORVD
evaluates the qualified AAR5 spline independently at the left/right rail-profile
reference stations. Each source is then interpolated independently to the
`100--250 m / 0.01 m` common station grid. Native time is compared by integer
sample index. No time or station shift, filtering, fitting, scaling, demeaning,
or result-dependent sign selection is permitted.

The main figures contain only ORVD and SIMPACK in each subplot. Current and
historical WRL remain explicit numerical controls; they are not drawn against a
different station observer merely to create a third line. Full trajectories,
statistics, source paths and performance logs stay under the untracked `tmp/`
tree. Only signed-off summary figures and concise tables may be copied into the
developer documentation; they are not product inputs or test baselines.
