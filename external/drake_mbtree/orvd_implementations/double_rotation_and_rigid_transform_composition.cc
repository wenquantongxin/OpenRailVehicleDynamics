// ORVD's own implementation of the four double-scalar pose compositions that
// drake/math/fast_pose_composition_functions.h declares.
//
// THIS FILE IS NOT UPSTREAM DRAKE SOURCE. Everything under ../drake/ is vendored
// upstream code; this directory is first-party ORVD code that satisfies vendored
// declarations. It lives beside the vendored tree because it is private support
// for that source boundary, not a public ORVD library. It is ours, and an upstream
// sync must not overwrite it with the upstream implementation.
//
// Why we implement it rather than vendor it: upstream's implementation dispatches
// through Google Highway, including hwy/foreach_target.h and hwy/highway.h
// unconditionally. That is an additional third-party dependency, and the
// product does not admit it. Extracting upstream's portable branch would be
// copying upstream code while calling it ours, so this is written from the
// definitions instead:
//
//   ComposeRR:    R_AC = R_AB R_BC
//   ComposeRinvR: R_AC = R_BAᵀ R_BC
//   ComposeXX:    R_AC = R_AB R_BC,  p_AC = p_AB + R_AB p_BC
//   ComposeXinvX: R_AC = R_BAᵀ R_BC, p_AC = R_BAᵀ (p_BC - p_BA)
//
// ComposeXX is deliberately not a 4x4 homogeneous matrix product: a rigid
// transform's last row is structurally constant, and multiplying it out spends
// arithmetic to recompute a row that is known.
//
// The declared contract allows the output to alias either input, including both
// at once. Every result is therefore computed into fixed-size locals first and
// written back only once all inputs have been read. No aliasing-analysis
// annotation is relied on: `noalias()` would be a promise the caller is
// explicitly allowed to break.
//
// ComposeRinvR and ComposeXinvX use the transpose rather than a general inverse
// because the declared precondition is that the input is a valid rotation, and
// for an orthonormal matrix the two coincide. The declared operation therefore
// needs no general inverse.
//
// Nothing here re-validates, re-orthonormalises or null-checks. Internal callers
// establish valid rotations and non-null outputs before entering this hot path.
#include "drake/math/fast_pose_composition_functions.h"

#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
namespace internal {

void ComposeRR(const RotationMatrix<double>& R_AB,
               const RotationMatrix<double>& R_BC,
               RotationMatrix<double>* R_AC) {
    const Eigen::Matrix3d composed_rotation_matrix =
        R_AB.matrix() * R_BC.matrix();
    *R_AC = RotationMatrix<double>::MakeUnchecked(composed_rotation_matrix);
}

void ComposeRinvR(const RotationMatrix<double>& R_BA,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC) {
    const Eigen::Matrix3d composed_rotation_matrix =
        R_BA.matrix().transpose() * R_BC.matrix();
    *R_AC = RotationMatrix<double>::MakeUnchecked(composed_rotation_matrix);
}

void ComposeXX(const RigidTransform<double>& X_AB,
               const RigidTransform<double>& X_BC,
               RigidTransform<double>* X_AC) {
    const Eigen::Matrix3d& R_AB = X_AB.rotation().matrix();
    const Eigen::Matrix3d composed_rotation_matrix =
        R_AB * X_BC.rotation().matrix();
    const Eigen::Vector3d composed_translation =
        X_AB.translation() + R_AB * X_BC.translation();
    X_AC->set(RotationMatrix<double>::MakeUnchecked(composed_rotation_matrix),
              composed_translation);
}

void ComposeXinvX(const RigidTransform<double>& X_BA,
                  const RigidTransform<double>& X_BC,
                  RigidTransform<double>* X_AC) {
    const Eigen::Matrix3d R_AB = X_BA.rotation().matrix().transpose();
    const Eigen::Matrix3d composed_rotation_matrix =
        R_AB * X_BC.rotation().matrix();
    const Eigen::Vector3d composed_translation =
        R_AB * (X_BC.translation() - X_BA.translation());
    X_AC->set(RotationMatrix<double>::MakeUnchecked(composed_rotation_matrix),
              composed_translation);
}

}  // namespace internal
}  // namespace math
}  // namespace drake
