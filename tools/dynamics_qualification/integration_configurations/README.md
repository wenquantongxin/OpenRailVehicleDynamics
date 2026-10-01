These files are explicit trial inputs for the existing qualification runners.
Their 25 μs step and Newton scales are trial values, not qualified vehicle defaults
or accuracy claims. Use `--integration-config PATH`; this option and a trailing
ODE qualification case are mutually exclusive.

The schema accepts `cvode_bdf2`, `cvode_bdf5`, `radau5`, `newmark` and `zhai`.
The ODE methods take physical-state tolerances. Newmark and Zhai require a positive
integer `step_size_nanoseconds`, converted to seconds once at the tool boundary.
The optional positive `maximum_internal_steps_per_advance` counts successful
internal steps per public advance and defaults to the library's 1,000,000-step
budget. It is not an accuracy parameter. Zhai has no Newton configuration.

Newmark requires all four scale families to be positive and finite, even if a
vehicle has no coordinates of one family. Translation scales use m, m/s, m/s²
and m/s²; angle scales use rad, rad/s, rad/s² and rad/s². In each case the fields
respectively describe position correction, coordinate velocity correction,
acceleration residual and acceleration reference for finite differences.
Quaternion values use stored quaternion units and the same time powers. Force
correction, residual and reference scales all use N.

`quaternion_scale_convention` must be `reference_norm_multiple`. All four
quaternion values are multiplied by the owning free body's stored norm at the
latest successful initialization, including explicit synchronization. The library
prepares the new reference and expanded scales together, commits both only after
initialization succeeds, and retains both after failure. Scales remain fixed
throughout each initialization epoch, including nonlinear solves and ordinary
advances. Synchronization recomputes the norm from the supplied state; floating
point rounding can therefore change the reference and expanded scales.

The library expands scalar families through actual joint types and coordinate
ranges. Expanded solver arrays stay private. Metadata records the declared
families, coordinate ownership and initial quaternion norms for interpreting
saved physical states. Newton scales are independent of physical-state ODE
tolerances. A Newton refinement multiplies all correction and residual family
values by 0.1 while keeping finite-difference references unchanged.
