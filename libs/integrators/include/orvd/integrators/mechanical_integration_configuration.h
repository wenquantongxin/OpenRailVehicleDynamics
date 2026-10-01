#pragma once

namespace orvd::integrators {

/// Dimensional Newton scales, not local integration error tolerances.
/// Every field must be positive and finite, including unused coordinate families.
struct CoordinateNewtonScales final {
    double position_correction{};
    double velocity_correction{};
    double acceleration_residual{};
    double acceleration_reference{};
};

/// Scales for series spring-damper force states, all in N.
struct SeriesForceNewtonScales final {
    double correction{};
    double residual{};
    double reference{};
};

struct NewmarkNewtonConfiguration final {
    int maximum_iterations{12};
    /// Free-body translation and prismatic joints: m, m/s, m/s², m/s².
    CoordinateNewtonScales translation;
    /// Revolute and Ball-RPY coordinates: rad, rad/s, rad/s², rad/s².
    CoordinateNewtonScales angle;
    /// Quaternion storage coordinates. All four scales are multiplied by each
    /// block's norm at the most recent successful initialization, including
    /// explicit synchronization. Synchronization recomputes that norm from the
    /// supplied state, so rounding may change the reference and expanded scales.
    /// The reference and scales are committed together only on success.
    CoordinateNewtonScales quaternion;
    /// Series spring-damper internal forces. Empty force state is supported.
    SeriesForceNewtonScales force;
};

struct NewmarkConfiguration final {
    /// Positive finite nominal and maximum step size, in seconds.
    double nominal_step_size_seconds{};
    NewmarkNewtonConfiguration nonlinear_solver;
};

struct ZhaiConfiguration final {
    /// Positive finite equal-grid step size, in seconds.
    double step_size_seconds{};
};

}  // namespace orvd::integrators
