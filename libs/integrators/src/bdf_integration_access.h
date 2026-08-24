#pragma once

/// @file
/// Source-tree-only construction and inspection for admitted BDF identities.

#include <memory>

#include "orvd/integrators/cvode_continuous_state_advancer.h"

namespace orvd::integrators::internal {

// This closed identity is deliberately unavailable from the installed include
// tree. The public constructors remain the admitted second-order default.
enum class MaximumBdfOrder : int {
    kSecond = 2,
    kFifth = 5,
};

[[nodiscard]] constexpr int MaximumBdfOrderValue(
    MaximumBdfOrder order) noexcept {
    return static_cast<int>(order);
}

/// Source-tree-only access used by focused tests and private qualification
/// tooling. It intentionally provides no arbitrary-order factory.
class BdfIntegrationAccess final {
   public:
    [[nodiscard]] static std::unique_ptr<CvodeContinuousStateAdvancer>
    MakeFifthOrderCvodeContinuousStateAdvancer(
        ContinuousStateRhs& rhs,
        double initial_time_seconds,
        Eigen::VectorXd initial_continuous_state,
        ContinuousStateErrorTolerances tolerances);

    [[nodiscard]] static int ConfiguredMaximumBdfOrder(
        const CvodeContinuousStateAdvancer& advancer);
    [[nodiscard]] static int LastBdfOrder(
        const CvodeContinuousStateAdvancer& advancer);
};

}  // namespace orvd::integrators::internal
