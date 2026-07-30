#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/// Abstract class that implements the basic logic of maintaining consequent
/// segments of time (delimited by `breaks`) to implement a trajectory that
/// is represented by simpler logic in each segment or "piece".
template <typename T>
class PiecewiseTrajectory : public Trajectory<T> {
 public:
  /// Minimum delta quantity used for comparing time.
  static constexpr double kEpsilonTime = std::numeric_limits<double>::epsilon();

  ~PiecewiseTrajectory() override;

  int get_number_of_segments() const;

  T start_time(int segment_number) const;
  using Trajectory<T>::start_time;  // Don't shadow the base class.

  T end_time(int segment_number) const;
  using Trajectory<T>::end_time;  // Don't shadow the base class.

  T duration(int segment_number) const;

  /**
   * Returns true iff `t >= getStartTime() && t <= getEndTime()`.
   */
  bool is_time_in_range(const T& t) const;

  int get_segment_index(const T& t) const;

  const std::vector<T>& get_segment_times() const;

  void segment_number_range_check(int segment_number) const;

 protected:
  // Final subclasses are allowed to make copy/move/assign public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseTrajectory);
  PiecewiseTrajectory() = default;

  /// @p breaks increments must be greater or equal to kEpsilonTime.
  explicit PiecewiseTrajectory(const std::vector<T>& breaks);

  // Trajectory overrides.
  T do_start_time() const override;
  T do_end_time() const override;

  bool SegmentTimesEqual(const PiecewiseTrajectory& b,
                         double tol = kEpsilonTime) const;

  const std::vector<T>& breaks() const { return breaks_; }
  std::vector<T>& get_mutable_breaks() { return breaks_; }

 private:
  int GetSegmentIndexRecursive(const T& time, int start, int end) const;

  std::vector<T> breaks_;
};

}  // namespace trajectories
}  // namespace drake

extern template class drake::trajectories::PiecewiseTrajectory<double>;
