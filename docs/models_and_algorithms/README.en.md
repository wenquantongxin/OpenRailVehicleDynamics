[中文](README.md)

# Theoretical Models and Computational Algorithms

This directory explains the rail-vehicle theory, mathematical models and computational algorithms adopted by ORVD, and maps quantities in the equations to their implementation in code. It is not a software user guide and does not carry API, configuration, test, exception, experiment or performance-qualification material.

## Boundary with technical documentation

| Directory | Responsibility |
|---|---|
| `models_and_algorithms/` | physical models, coordinates and signs, mathematical derivations, discrete algorithms, theoretical approximations and their implementation in code |
| `design/` | software architecture, module relationships, interface contracts and runtime ownership |
| `engineering/` | APIs, configuration, input formats, failure behaviour, tests and engineering discipline |
| `performance/` | caches, workspaces, parallel scheduling, hardware use, timing and optimisation qualification |
| `planning/` | implementation order, migration records and project decisions |

Computational acceleration belongs here only when it changes the mathematical computation itself, such as an algebraically equivalent transformation, a reduction in the complexity of a discrete algorithm, or a controlled approximation with an error budget. Cache keys, memory reuse, thread scheduling, allocation behaviour and machine timing belong to technical documentation.

A theory document may state a model's mathematical assumptions and conditions of applicability, but it does not define the model through test coverage, experimental cases or qualification results. Theory not yet implemented in code is marked **Theory only**. Implemented material must state the formula and discretisation actually used by ORVD.

## Topic categories

| Category | Content |
|---|---|
| `track_geometry/` | horizontal and vertical alignment, superelevation, track frames, station projection and vertical profiles |
| `track_irregularity_spectra/` | track-irregularity PSD, finite spatial bands, random realisations and multi-direction correlation |
| `wheel_rail_contact/` | profiles and interpolation, pose reduction, contact geometry, normal force, creepage, Kalker coefficients, FASTSIM and wrench assembly |
| `vehicle_dynamics/` | multibody dynamics, vehicle topology, suspension force elements, rigid wheelsets and independently rotating wheels |
| `control_and_estimation/` | mathematical models of control laws, state estimation, sampling and actuators |
| `numerical_methods/` | time integration, nonlinear solution, numerical Jacobians, error control and stability |

## Form of a theory document

- A Chinese primary document and its English namesake marked `.en` before the extension are stored together, carry equivalent content, and link to each other on the first line.
- GitHub Markdown is the publication format; inline mathematics uses `$...$` and display mathematics uses standalone `$$` blocks.
- Structure follows the subject, but scope and implementation status, notation, theoretical model, computational algorithm, code realisation and theoretical assumptions must be identifiable.
- “Code realisation” explains how the equations become the core data structures, functions and evaluation order. It is not an API list, configuration reference, exception catalogue or test index.
- A constant appears only when it defines part of a model or algorithm, and its mathematical role must be explained; a table of source literals is not a configuration reference.
- Derivations, dimensional checks, limiting cases and necessary independent recomputation are part of theoretical writing and are not constrained to source-code spelling.
- When a theoretical source must be named, link the original paper, monograph or authoritative review briefly at the relevant point in the text. No central or per-document bibliography is maintained for now. Source code proves what ORVD adopts; it does not replace the theory's provenance.
- Test evidence, experimental results, qualification records, timing results, commit history and migration process do not belong in this directory.

All documents share [Conventions and Notation](CONVENTIONS_AND_NOTATION.en.md).

## Document index

### Shared basis

- [Conventions and Notation](CONVENTIONS_AND_NOTATION.en.md): the track inertial and track frames, signs, station and arc length, pose, state blocks, wrenches and core Chinese-English terminology.

### Track geometry

- [Track geometry and track frames](track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md): scalar profiles, the Bloss/Hermite curvature transition, quintic seams, planar integration, track frames, tangent continuation and local station projection.
- [Track vertical profile modelling and its three-dimensional coupling](track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.en.md): constant-grade segments, PL2, CIR, vertical seams and their coupling with horizontal alignment and superelevation.

### Track irregularity

- [Track irregularity spectra and their spatial random realisations](track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.en.md): spatial frequency, FRA/AAR spectra, finite bands, random realisation and multi-direction relationships.

### Wheel-rail contact

- [Profiles and interpolants](wheel_rail_contact/PROFILES_AND_INTERPOLANTS.en.md): side resolution of profiles, natural cubic splines, shape-preserving cubic interpolation, arc length, equal-arc-length resampling and the rail gauge datum.
- [Wheel-rail pose reduction and irregularity input](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md): reduction of a three-dimensional wheelset state to four pose scalars, local frames, irregularity inputs and X-Z-Y attitude resolution.
- [Contact geometry](wheel_rail_contact/CONTACT_GEOMETRY.en.md): visible outline, envelope, contact islands, per-island quadrature, contact angles and the three-dimensional longitudinal chord.
- [Normal contact force](wheel_rail_contact/NORMAL_CONTACT_FORCE.en.md): equal-area circular segment, equivalent penetration, the Hertz solution, elliptic integral, damping and longitudinal baseline.
- [Creepages and the contact frame](wheel_rail_contact/CREEPAGE_AND_CONTACT_FRAME.en.md): contact frame, reference speed, the three creepages, normal approach speed and rolling-radius convention.
- [Kalker linear creep coefficients](wheel_rail_contact/KALKER_COEFFICIENTS.en.md): finite coefficient tables, interpolation in Poisson ratio and semi-axis ratio, and slender-ellipse asymptotics.
- [Tangential contact force: FASTSIM](wheel_rail_contact/TANGENTIAL_CONTACT_FASTSIM.en.md): strip marching, stress accumulation, pressure distribution, adhesion-slip boundary, spin refinement and the falling friction law.
- [Single-wheel contact model assembly and paired wrench](wheel_rail_contact/CONTACT_MODEL_ASSEMBLY_AND_WRENCH.en.md): physical assembly of the contact chain, material reference point, wheel-side application point, coordinate transformations and the paired wrench.

### Numerical methods

- [BDF, Radau5, Newmark and Zhai time-integration methods](numerical_methods/TIME_INTEGRATION_METHODS.en.md): discrete formulas, single-step algorithms, error and stability, and the conditions under which each method applies to the ORVD state structure.
