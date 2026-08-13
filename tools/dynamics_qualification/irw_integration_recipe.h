#pragma once

namespace orvd::dynamics_qualification::internal {

// One private integration recipe shared by passive and controlled IRW
// qualification runs. It is not installed or exposed as an integrator API.
inline constexpr double kIrwRelativeTolerance = 1.0e-6;
inline constexpr double kIrwPositionAbsoluteTolerance = 1.0e-6;
inline constexpr double kIrwVelocityAbsoluteTolerance = 1.0e-5;
inline constexpr double kIrwSeriesForceAbsoluteToleranceNewtons = 1.0e-6;

}  // namespace orvd::dynamics_qualification::internal
