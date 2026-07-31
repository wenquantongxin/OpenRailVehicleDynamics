#include "orvd/integrators/continuous_state_advancer.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::integrators {

ContinuousStateErrorTolerances::ContinuousStateErrorTolerances(
    double relative_tolerance,
    Eigen::VectorXd component_absolute_tolerances)
    : relative_tolerance_(relative_tolerance),
      component_absolute_tolerances_(
          std::move(component_absolute_tolerances)) {
    if (!std::isfinite(relative_tolerance_) || relative_tolerance_ < 0.0) {
        throw std::invalid_argument(
            "continuous-state tolerances: relative tolerance must be finite "
            "and non-negative");
    }
    for (Eigen::Index i = 0; i < component_absolute_tolerances_.size(); ++i) {
        const double value = component_absolute_tolerances_[i];
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                "continuous-state tolerances: absolute tolerance at entry " +
                std::to_string(i) + " must be finite and positive");
        }
    }
}

}  // namespace orvd::integrators
