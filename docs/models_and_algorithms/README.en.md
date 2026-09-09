[中文](README.md)

# Theoretical models and computational algorithms

This directory explains the rail-vehicle theory, mathematical models and computational algorithms adopted by ORVD, and maps quantities in the equations to their implementation in code. It is not a software user guide and does not carry API, configuration, test, exception, experiment or performance-qualification material.

## Boundary with technical documentation

| Directory | Responsibility |
|---|---|
| `models_and_algorithms/` | physical models, coordinates and signs, mathematical derivations, discrete algorithms, theoretical approximations and their implementation in code |
| `design/` | software architecture, module relationships, interface contracts and runtime ownership |
| `engineering/` | APIs, configuration, input formats, failure behaviour, tests and engineering discipline |
| `performance/` | caches, workspaces, parallel scheduling, hardware use, timing and optimization qualification |
| `planning/` | implementation order, migration records and project decisions |

Computational acceleration belongs here only when it changes the mathematical computation itself, such as an algebraically equivalent transformation, a reduction in the complexity of a discrete algorithm or a controlled approximation with an error budget. Cache keys, memory reuse, thread scheduling, allocation behaviour and machine timing belong to technical documentation.

A theory document may state a model's mathematical assumptions and conditions of applicability, but it does not define the model through test coverage, experimental cases or qualification results. Theory not yet implemented in code is marked **Theory only**. Implemented material must state the formula and discretization actually used by ORVD.

## Published topics

| Category | Content |
|---|---|
| `track_geometry/` | horizontal and vertical alignment, superelevation, track frames, station projection and vertical profiles |
| `track_irregularity_spectra/` | track-irregularity PSD, finite spatial bands, random realizations and multi-direction correlation |
| `wheel_rail_contact/` | profiles and interpolation, pose reduction, contact geometry, normal force, creepage, Kalker coefficients, FASTSIM, wrench assembly and contact-force-plan kinematics |
| `force_elements/` | connection kinematics and wrenches, translational spring-damper, roll couple, series Maxwell element, saturated piecewise-linear damper and half-angle midpoint RPY bushing |
| `vehicle_dynamics/` | multibody equations of motion, articulated-body forward dynamics, assembly of the complete right-hand side, and startup state construction |
| `numerical_methods/` | time integration, implicit nonlinear iteration, error control and stability |

## Form of a theory document

- A Chinese primary document and its English namesake marked `.en` before the extension are stored together, carry equivalent content and link to each other on the first line.
- GitHub Markdown is the publication format; inline mathematics uses `$...$` and display mathematics uses standalone `$$` blocks.
- Structure follows the subject, but scope and implementation status, notation, theoretical model, computational algorithm, code realization and theoretical assumptions must be identifiable.
- “Code realization” explains how the equations become the core data structures, functions and evaluation order. It is not an API list, configuration reference, exception catalogue or test index.
- A constant appears only when it defines part of a model or algorithm, and its mathematical role must be explained; a table of source literals is not a configuration reference.
- When a theoretical source must be named, link the original paper, monograph or authoritative review briefly at the relevant point in the text. No central or per-document bibliography is maintained for now. Source code proves what ORVD adopts; it does not replace the theory's provenance.
- Test evidence, experimental results, qualification records, timing results, commit history and migration process do not belong in this directory.

All documents share [Conventions and notation](CONVENTIONS_AND_NOTATION.en.md).

## Document index

### Shared basis

- [Conventions and notation](CONVENTIONS_AND_NOTATION.en.md): the track inertial frame and the track frame, signs, station and arc length, pose, state blocks, wrenches and core Chinese-English terminology.

### Line geometry

- [Line geometry and track frames](track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md): scalar profiles, the Bloss/Hermite curvature transition, quintic seams, planar integration, track frames, tangent continuation and local station projection.
- [Track vertical profile modelling and its three-dimensional coupling](track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.en.md): constant-grade segments, PL2, CIR, vertical seams and their coupling with horizontal alignment and superelevation.

### Track irregularity

- [Track-irregularity spectra and their spatial random realization](track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.en.md): spatial frequency, FRA/AAR spectra, finite bands, random realization and multi-direction relationships.

### Wheel-rail contact

- [Profiles and interpolants](wheel_rail_contact/PROFILES_AND_INTERPOLANTS.en.md): side resolution of profiles, natural cubic splines, shape-preserving cubic interpolation, arc length, equal-arc-length resampling and the rail gauge datum.
- [Wheel-rail pose reduction and irregularity inputs](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md): reduction of a three-dimensional wheelset state to four pose scalars, local frames, irregularity inputs and X-Z-Y attitude resolution.
- [Contact geometry](wheel_rail_contact/CONTACT_GEOMETRY.en.md): visible outline, envelope, contact islands, per-island quadrature, common-normal and contact-frame angles, and the three-dimensional longitudinal chord.
- [Normal contact force](wheel_rail_contact/NORMAL_CONTACT_FORCE.en.md): equal-area circular segment, equivalent penetration, the Hertz solution, elliptic integral, damping and longitudinal baseline.
- [Creepages and the contact frame](wheel_rail_contact/CREEPAGE_AND_CONTACT_FRAME.en.md): contact frame, reference speed, the three creepages, normal approach speed and rolling-radius convention.
- [Kalker linear creepage coefficients](wheel_rail_contact/KALKER_COEFFICIENTS.en.md): finite coefficient tables, interpolation in Poisson ratio, interpolation in semi-axis ratio and slender-ellipse asymptotics.
- [Tangential contact force: FASTSIM strip marching](wheel_rail_contact/TANGENTIAL_CONTACT_FASTSIM.en.md): strip marching, stress accumulation, pressure distribution, adhesion-slip boundary, spin refinement and the falling friction law.
- [Single-wheel contact-model assembly and paired wrench](wheel_rail_contact/CONTACT_MODEL_ASSEMBLY_AND_WRENCH.en.md): physical assembly of the contact chain, material reference point, wheel-side application point, coordinate transformations and the paired wrench.
- [Contact force plan kinematics](wheel_rail_contact/CONTACT_FORCE_PLAN_KINEMATICS.en.md): seeded local-branch projection of the carrier station, pose and rates in the track frame, the spin-free geometric attitude, interface kinematics of the two carrier types, rigid motion of the profile and irregularity sampling, rail-profile placement, and combination of the per-patch wrenches into one equivalent body wrench acting on the wheel body.

### Force elements

- [Force-element kinematics and spatial wrenches](force_elements/FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md): relative position, relative attitude, relative velocity with transport term and relative angular velocity of the two ends, the three paired-wrench application schemes, reduction-point rules, power identities and the organizing principle.
- [Three-axis translational spring-damper element](force_elements/TRANSLATIONAL_SPRING_DAMPER.en.md): parallel spring-damper pairs on the three reference-end axes with a nominal force, endpoint force pair with support moment, stored energy and dissipation, and the attitude dependence of the diagonal law in space.
- [Roll spring-damper couple](force_elements/ROLL_SPRING_DAMPER_COUPLE.en.md): matrix-entry roll measure, relative-angular-velocity component, pure couple pair, stored energy under pure roll and properties under coupled rotation.
- [Series spring-viscous-damper element](force_elements/SERIES_SPRING_VISCOUS_DAMPER.en.md): Maxwell force equation, relaxation time, analytical responses and the coupling of the force state to the system continuous state.
- [Odd-symmetric saturated piecewise-linear damper](force_elements/SATURATED_PIECEWISE_LINEAR_DAMPER.en.md): piecewise-linear force curve on the non-negative half-axis, constant continuation beyond the last node, odd extension, consequences of the three domain requirements and the dissipation potential.
- [Half-angle midpoint RPY bushing](force_elements/HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md): half-angle intermediate frame, midpoint relative material velocity, space-XYZ angle extraction and rate map, physical moment by power conjugacy and the midpoint wrench pair.

### Vehicle dynamics

- [Multibody equations of motion](vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md): rigid-body tree and generalized coordinates, rate maps of quaternions and the Ball-RPY joint, spatial velocities and the shift of body wrenches to the body origin, power conjugacy and generalized forces, equations of motion and on-demand assembly of the mass matrix, the three-pass articulated-body forward dynamics and assembly of the complete right-hand side.
- [Startup state assembly](vehicle_dynamics/STARTUP_STATE_ASSEMBLY.en.md): the physically meaningful initial quantities of a startup record, the station as a sum of three terms and attitude and position in the complete track frame at each body's station, change of basis of inertial velocities and the common-spin generation rule, joint coordinates and series internal forces, and accompanying non-state quantities such as nominal forces and the rail-profile vertical datum.

### Numerical methods

- [BDF, Radau5, Newmark and Zhai time-integration methods](numerical_methods/TIME_INTEGRATION_METHODS.en.md): discrete formulas, one-step and multistep advancement algorithms, formation of the finite-difference Jacobian, error and stability, as well as the conditions under which each method applies to the ORVD state structure.
