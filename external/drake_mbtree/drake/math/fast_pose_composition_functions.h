#pragma once

namespace drake {
namespace math {

// We must forward-declare these classes to avoid a circular dependency.
#ifndef DRAKE_DOXYGEN_CXX
template <typename>
class RotationMatrix;
template <typename>
class RigidTransform;
#endif

namespace internal {

/* Declarations for low-level functions for composing rotation matrices and rigid
transforms of double. ORVD implements these from their mathematical definitions in
orvd_implementations/; upstream's SIMD dispatch is not part of this build and no
performance claim is made for them. */

/* Composes two RotationMatrix<double> objects, resulting in a new
RotationMatrix.

Here we calculate `R_AC = R_AB * R_BC`. It is OK for R_AC to overlap
with one or both inputs.
@pre R_AC is not null. */
void ComposeRR(const RotationMatrix<double>& R_AB,
               const RotationMatrix<double>& R_BC,
               RotationMatrix<double>* R_AC);

/* Composes the inverse of a RotationMatrix<double> object with another
(non-inverted) RotationMatrix<double>, resulting in a new
RotationMatrix.

@note A valid RotationMatrix is orthonormal, and the inverse of an orthonormal
matrix is just its transpose. This function assumes orthonormality and hence
simply multiplies the transpose of its first argument by the second.

Here we calculate `R_AC = R_BA⁻¹ * R_BC`. It is OK for R_AC to overlap with one
or both inputs.
@pre R_AC is not null. */
void ComposeRinvR(const RotationMatrix<double>& R_BA,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC);

/* Composes two RigidTransform<double> objects, resulting
in a new RigidTransform.

@note This function is specialized for RigidTransforms and is not just a matrix
multiply.

Here we calculate `X_AC = X_AB * X_BC`. It is OK for X_AC to overlap with one or
both inputs.
@pre X_AC is not null. */
void ComposeXX(const RigidTransform<double>& X_AB,
               const RigidTransform<double>& X_BC,
               RigidTransform<double>* X_AC);

/* Composes the inverse of a RigidTransform<double> object with another
(non-inverted) RigidTransform<double>, resulting in a new
RigidTransform.

@note This function is specialized for RigidTransforms and is not just a matrix
multiply.

Here we calculate `X_AC = X_BA⁻¹ * X_BC`. It is OK for X_AC to overlap with one
or both inputs.
@pre X_AC is not null. */
void ComposeXinvX(const RigidTransform<double>& X_BA,
                  const RigidTransform<double>& X_BC,
                  RigidTransform<double>* X_AC);

}  // namespace internal
}  // namespace math
}  // namespace drake
