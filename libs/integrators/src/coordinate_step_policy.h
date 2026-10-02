#pragma once

#include "coordinate_second_order_problem.h"

namespace orvd::integrators::internal {

// These policies own only step selection, never states or system callbacks.
class FixedCoordinateStepPolicy final {
   public:
    explicit FixedCoordinateStepPolicy(double h) : h_(h) {}
    [[nodiscard]] double step_size() const { return h_; }
    template <typename Core>
    static bool TryAdvance(Core& core, double h, double end) {
        core.AdvanceOneStep(h, end);
        return true;
    }
    bool ReduceAfterRejection(double) { return false; }
    bool AcceptedStep() { return false; }
    void Reset() {}

   private:
    const double h_;
};

class NewmarkRecoveryStepPolicy final {
   public:
    explicit NewmarkRecoveryStepPolicy(double h) : nominal_h_(h), h_(h) {}
    [[nodiscard]] double step_size() const { return h_; }
    template <typename Core>
    static bool TryAdvance(Core& core, double h, double end) {
        return core.AdvanceOneStep(h, end) == Core::StepResult::kAccepted;
    }
    bool ReduceAfterRejection(double attempted_h) {
        success_streak_ = 0;
        const double smaller = 0.5 * attempted_h;
        // This also bounds retrying when nominal_h_/1024 rounds to zero.
        if (!(smaller > 0.0 && smaller < attempted_h)) {
            throw CoordinateIntegrationFailure(
                CoordinateIntegrationFailure::Reason::kStepSizeUnderflow,
                "Newmark: no positive representable reduced step");
        }
        if (smaller < nominal_h_ * kMinimumRetryFraction) return false;
        h_ = smaller;
        return true;
    }
    // Called only after physical publication. A change requires reanchoring.
    bool AcceptedStep() {
        if (h_ == nominal_h_) {
            success_streak_ = 0;
            return false;
        }
        if (++success_streak_ < 2) return false;
        h_ = h_ >= 0.5 * nominal_h_ ? nominal_h_ : 2.0 * h_;
        success_streak_ = 0;
        return true;
    }
    void Reset() {
        h_ = nominal_h_;
        success_streak_ = 0;
    }

   private:
    static constexpr double kMinimumRetryFraction = 1.0 / 1024.0;
    const double nominal_h_;
    double h_;
    unsigned success_streak_{};
};

}  // namespace orvd::integrators::internal
