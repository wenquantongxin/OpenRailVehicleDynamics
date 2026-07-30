// Contract check for ORVD's four double-scalar pose compositions.
//
// These four functions are declared by the vendored
// drake/math/fast_pose_composition_functions.h and implemented by ORVD in
// external/drake_mbtree/orvd_implementations/. This program links the real
// symbols — drake::math::internal::ComposeRR and its three siblings — so a
// change to the implementation is what this checks, not a copy of it.
//
// Two properties are checked for each function:
//
//   the algebra   R_AC = R_AB R_BC, R_AC = R_BAᵀ R_BC,
//                 (R_AC, p_AC) = (R_AB R_BC, p_AB + R_AB p_BC),
//                 (R_AC, p_AC) = (R_BAᵀ R_BC, R_BAᵀ (p_BC − p_BA));
//
//   the aliasing contract, which the declaration states: the output may alias
//                 either input, or both at once.
//
// Every expected value is computed with bare Eigen from the four definitions
// above, and computed BEFORE the call, because a call whose output aliases an
// input overwrites what the expectation would otherwise be read from. The
// expectations deliberately do not use RotationMatrix::operator*,
// RigidTransform::operator* or InvertAndCompose: those route back through the
// very symbols under test, so an implementation and its expectation would agree
// by construction and the check would certify nothing.
//
// The tolerance is not the project's 1e-3 engineering caliber. That caliber is
// for comparing a physical quantity across two implementations of a model. This
// is one algebraic identity evaluated two ways in the same arithmetic, so the
// honest bar is a small multiple of the representation's own resolution:
// 64 * epsilon scaled by the magnitudes involved. A wrong implementation — a
// transpose in the wrong place, a missing translation term, a factor of two —
// misses that bar by many orders of magnitude.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/math/fast_pose_composition_functions.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace {

using drake::math::RigidTransform;
using drake::math::RotationMatrix;

// Upstream's shorthand for the four aliasing forms is ABO / ABA / ABB / AAA,
// where the third letter names the output. The names below say the same thing
// without requiring the reader to know the shorthand.
enum class AliasingForm {
  kDistinctOutput,
  kOutputAliasesFirstInput,
  kOutputAliasesSecondInput,
  kAllArgumentsAliasSameObject,
};

const char* AliasingFormName(AliasingForm form) {
  switch (form) {
    case AliasingForm::kDistinctOutput:
      return "DistinctOutput";
    case AliasingForm::kOutputAliasesFirstInput:
      return "OutputAliasesFirstInput";
    case AliasingForm::kOutputAliasesSecondInput:
      return "OutputAliasesSecondInput";
    case AliasingForm::kAllArgumentsAliasSameObject:
      return "AllArgumentsAliasSameObject";
  }
  std::unreachable();
}

int failure_count = 0;

// A small multiple of the representation's resolution, scaled by the magnitudes
// that entered the computation. Not a fixed absolute number: a translation of
// 1e6 metres cannot be checked against the same bound as one of 1e-3.
double ToleranceFor(double expected_magnitude, double input_scale) {
  constexpr double kEpsilonMultiple = 64.0;
  const double scale = std::max({1.0, input_scale, expected_magnitude});
  return kEpsilonMultiple * std::numeric_limits<double>::epsilon() * scale;
}

void ExpectMatricesMatch(const std::string& composition_name, AliasingForm form,
                         const Eigen::Matrix3d& actual,
                         const Eigen::Matrix3d& expected, double input_scale) {
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const double difference =
          std::abs(actual(row, column) - expected(row, column));
      const double tolerance =
          ToleranceFor(std::abs(expected(row, column)), input_scale);
      if (!(difference <= tolerance)) {
        std::printf(
            "FAIL %s [%s]: rotation element (%d,%d) is %.17g, expected %.17g"
            " (difference %.3e exceeds %.3e)\n",
            composition_name.c_str(), AliasingFormName(form), row, column,
            actual(row, column), expected(row, column), difference, tolerance);
        ++failure_count;
      }
    }
  }
}

void ExpectVectorsMatch(const std::string& composition_name, AliasingForm form,
                        const Eigen::Vector3d& actual,
                        const Eigen::Vector3d& expected, double input_scale) {
  for (int component = 0; component < 3; ++component) {
    const double difference =
        std::abs(actual(component) - expected(component));
    const double tolerance =
        ToleranceFor(std::abs(expected(component)), input_scale);
    if (!(difference <= tolerance)) {
      std::printf(
          "FAIL %s [%s]: translation component %d is %.17g, expected %.17g"
          " (difference %.3e exceeds %.3e)\n",
          composition_name.c_str(), AliasingFormName(form), component,
          actual(component),
          expected(component), difference, tolerance);
      ++failure_count;
    }
  }
}

// Built with bare Eigen so that constructing the inputs cannot route through the
// composition being checked. Two rotations about non-parallel axes by unequal
// angles: they do not commute, so an implementation that swaps its operands
// fails rather than coincidentally agreeing.
Eigen::Matrix3d FirstRotationMatrix() {
  return Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.0, 0.0, 1.0))
      .toRotationMatrix();
}

Eigen::Matrix3d SecondRotationMatrix() {
  return Eigen::AngleAxisd(-1.1, Eigen::Vector3d(1.0, 2.0, -2.0).normalized())
      .toRotationMatrix();
}

// Non-zero, mutually distinct, and not parallel to either rotation axis, so that
// a dropped or misplaced translation term changes the result.
Eigen::Vector3d FirstTranslation() { return {1.5, -0.25, 3.0}; }
Eigen::Vector3d SecondTranslation() { return {-0.75, 2.25, 0.5}; }

double TranslationInputScale() {
  return std::max({FirstTranslation().cwiseAbs().maxCoeff(),
                   SecondTranslation().cwiseAbs().maxCoeff(), 1.0});
}

// ---------------------------------------------------------------------------
// R_AC = R_AB R_BC
void CheckComposeRR(AliasingForm form) {
  const bool arguments_alias =
      form == AliasingForm::kAllArgumentsAliasSameObject;
  const Eigen::Matrix3d first = FirstRotationMatrix();
  const Eigen::Matrix3d second = arguments_alias ? first : SecondRotationMatrix();
  const Eigen::Matrix3d expected = first * second;

  RotationMatrix<double> R_AB = RotationMatrix<double>::MakeUnchecked(first);
  RotationMatrix<double> R_BC = RotationMatrix<double>::MakeUnchecked(second);
  RotationMatrix<double> separate_output =
      RotationMatrix<double>::MakeUnchecked(Eigen::Matrix3d::Identity());

  RotationMatrix<double>* output = nullptr;
  switch (form) {
    case AliasingForm::kDistinctOutput:
      output = &separate_output;
      break;
    case AliasingForm::kOutputAliasesFirstInput:
    case AliasingForm::kAllArgumentsAliasSameObject:
      output = &R_AB;
      break;
    case AliasingForm::kOutputAliasesSecondInput:
      output = &R_BC;
      break;
  }
  if (arguments_alias) {
    drake::math::internal::ComposeRR(R_AB, R_AB, output);
  } else {
    drake::math::internal::ComposeRR(R_AB, R_BC, output);
  }
  ExpectMatricesMatch("ComposeRR", form, output->matrix(), expected, 1.0);
}

// R_AC = R_BAᵀ R_BC
void CheckComposeRinvR(AliasingForm form) {
  const bool arguments_alias =
      form == AliasingForm::kAllArgumentsAliasSameObject;
  const Eigen::Matrix3d first = FirstRotationMatrix();
  const Eigen::Matrix3d second = arguments_alias ? first : SecondRotationMatrix();
  const Eigen::Matrix3d expected = first.transpose() * second;

  RotationMatrix<double> R_BA = RotationMatrix<double>::MakeUnchecked(first);
  RotationMatrix<double> R_BC = RotationMatrix<double>::MakeUnchecked(second);
  RotationMatrix<double> separate_output =
      RotationMatrix<double>::MakeUnchecked(Eigen::Matrix3d::Identity());

  RotationMatrix<double>* output = nullptr;
  switch (form) {
    case AliasingForm::kDistinctOutput:
      output = &separate_output;
      break;
    case AliasingForm::kOutputAliasesFirstInput:
    case AliasingForm::kAllArgumentsAliasSameObject:
      output = &R_BA;
      break;
    case AliasingForm::kOutputAliasesSecondInput:
      output = &R_BC;
      break;
  }
  if (arguments_alias) {
    drake::math::internal::ComposeRinvR(R_BA, R_BA, output);
  } else {
    drake::math::internal::ComposeRinvR(R_BA, R_BC, output);
  }
  ExpectMatricesMatch("ComposeRinvR", form, output->matrix(), expected, 1.0);
}

// R_AC = R_AB R_BC,  p_AC = p_AB + R_AB p_BC
void CheckComposeXX(AliasingForm form) {
  const bool arguments_alias =
      form == AliasingForm::kAllArgumentsAliasSameObject;
  const Eigen::Matrix3d first_rotation = FirstRotationMatrix();
  const Eigen::Vector3d first_translation = FirstTranslation();
  const Eigen::Matrix3d second_rotation =
      arguments_alias ? first_rotation : SecondRotationMatrix();
  const Eigen::Vector3d second_translation =
      arguments_alias ? first_translation : SecondTranslation();
  const Eigen::Matrix3d expected_rotation = first_rotation * second_rotation;
  const Eigen::Vector3d expected_translation =
      first_translation + first_rotation * second_translation;

  RigidTransform<double> X_AB(
      RotationMatrix<double>::MakeUnchecked(first_rotation), first_translation);
  RigidTransform<double> X_BC(
      RotationMatrix<double>::MakeUnchecked(second_rotation),
      second_translation);
  RigidTransform<double> separate_output;

  RigidTransform<double>* output = nullptr;
  switch (form) {
    case AliasingForm::kDistinctOutput:
      output = &separate_output;
      break;
    case AliasingForm::kOutputAliasesFirstInput:
    case AliasingForm::kAllArgumentsAliasSameObject:
      output = &X_AB;
      break;
    case AliasingForm::kOutputAliasesSecondInput:
      output = &X_BC;
      break;
  }
  if (arguments_alias) {
    drake::math::internal::ComposeXX(X_AB, X_AB, output);
  } else {
    drake::math::internal::ComposeXX(X_AB, X_BC, output);
  }
  ExpectMatricesMatch("ComposeXX", form, output->rotation().matrix(),
                      expected_rotation, 1.0);
  ExpectVectorsMatch("ComposeXX", form, output->translation(),
                     expected_translation, TranslationInputScale());
}

// R_AC = R_BAᵀ R_BC,  p_AC = R_BAᵀ (p_BC − p_BA)
void CheckComposeXinvX(AliasingForm form) {
  const bool arguments_alias =
      form == AliasingForm::kAllArgumentsAliasSameObject;
  const Eigen::Matrix3d first_rotation = FirstRotationMatrix();
  const Eigen::Vector3d first_translation = FirstTranslation();
  const Eigen::Matrix3d second_rotation =
      arguments_alias ? first_rotation : SecondRotationMatrix();
  const Eigen::Vector3d second_translation =
      arguments_alias ? first_translation : SecondTranslation();
  const Eigen::Matrix3d expected_rotation =
      first_rotation.transpose() * second_rotation;
  const Eigen::Vector3d expected_translation =
      first_rotation.transpose() * (second_translation - first_translation);

  RigidTransform<double> X_BA(
      RotationMatrix<double>::MakeUnchecked(first_rotation), first_translation);
  RigidTransform<double> X_BC(
      RotationMatrix<double>::MakeUnchecked(second_rotation),
      second_translation);
  RigidTransform<double> separate_output;

  RigidTransform<double>* output = nullptr;
  switch (form) {
    case AliasingForm::kDistinctOutput:
      output = &separate_output;
      break;
    case AliasingForm::kOutputAliasesFirstInput:
    case AliasingForm::kAllArgumentsAliasSameObject:
      output = &X_BA;
      break;
    case AliasingForm::kOutputAliasesSecondInput:
      output = &X_BC;
      break;
  }
  if (arguments_alias) {
    drake::math::internal::ComposeXinvX(X_BA, X_BA, output);
  } else {
    drake::math::internal::ComposeXinvX(X_BA, X_BC, output);
  }
  ExpectMatricesMatch("ComposeXinvX", form, output->rotation().matrix(),
                      expected_rotation, 1.0);
  ExpectVectorsMatch("ComposeXinvX", form, output->translation(),
                     expected_translation, TranslationInputScale());
}

}  // namespace

int main() {
  constexpr AliasingForm kAliasingForms[] = {
      AliasingForm::kDistinctOutput,
      AliasingForm::kOutputAliasesFirstInput,
      AliasingForm::kOutputAliasesSecondInput,
      AliasingForm::kAllArgumentsAliasSameObject,
  };
  for (const AliasingForm form : kAliasingForms) {
    CheckComposeRR(form);
    CheckComposeRinvR(form);
    CheckComposeXX(form);
    CheckComposeXinvX(form);
  }
  if (failure_count > 0) {
    std::printf("%d pose composition check(s) failed\n", failure_count);
    return 1;
  }
  std::printf(
      "four pose compositions satisfy their algebra in all four aliasing"
      " forms\n");
  return 0;
}
