#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {
namespace internal {
/*
 * The type of the Eigen linear solver that LinearSolver holds. For example
 * EigenLinearSolver<Eigen::LLT, Eigen::Matrix3d> is
 * Eigen::LLT<Eigen::Matrix3d>.
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA>
using EigenLinearSolver = LinearSolverType<Eigen::Matrix<
    typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
    DerivedA::ColsAtCompileTime, Eigen::ColMajor,
    DerivedA::MaxRowsAtCompileTime, DerivedA::MaxColsAtCompileTime>>;
}  // namespace internal

/**
 * Solves a linear system of equations A*x=b.
 *
 * Holding the decomposition separately from the solve lets a caller factor once
 * and then solve for several right-hand sides, which is what the articulated
 * body algorithm does with the hinge inertia of each mobilized body.
 *
 * @tparam LinearSolverType The type of linear solver, for example Eigen::LLT.
 * Notice that this just specifies the solver type (such as Eigen::LLT), not the
 * matrix type (like Eigen::LLT<Eigen::Matrix2d>). All Eigen solvers we care
 * about are templated on the matrix type. Some are further templated on
 * configuration ints. The int... will account for zero or more of these ints,
 * providing a common interface for both types of solvers.
 * @tparam DerivedA An Eigen Matrix.
 *
 * Here is an example code.
 * @code{cc}
 * Eigen::Matrix2d A;
 * A << 1, 2, 2, 5;
 * LinearSolver<Eigen::LLT, Eigen::Matrix2d> solver(A);
 * Eigen::Vector2d b;
 * b << 3, 4;
 * Eigen::Vector2d x = solver.Solve(b);
 * @endcode
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA>
class LinearSolver {
 public:
  using SolverType = internal::EigenLinearSolver<LinearSolverType, DerivedA>;

  /**
   * The return type of the Solve() function, which is the return type of
   * Eigen's own Decomposition::solve(). Returning Eigen's expression rather
   * than a concrete Eigen::Matrix avoids a copy and a heap allocation.
   */
  template <typename DerivedB>
  using SolutionType = Eigen::Solve<SolverType, DerivedB>;

  /** Default constructor. Constructs an empty linear solver. */
  LinearSolver() : eigen_linear_solver_() {}

  explicit LinearSolver(const Eigen::MatrixBase<DerivedA>& A)
      : eigen_linear_solver_{A} {}

  /** Solves system A*x = b. */
  template <typename DerivedB>
  SolutionType<DerivedB> Solve(const Eigen::MatrixBase<DerivedB>& b) const {
    return eigen_linear_solver_.solve(b);
  }

  /** Getter for the Eigen linear solver. */
  const SolverType& eigen_linear_solver() const { return eigen_linear_solver_; }

 private:
  SolverType eigen_linear_solver_;
};

}  // namespace math
}  // namespace drake
