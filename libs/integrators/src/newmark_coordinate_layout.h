#pragma once

#include <vector>

#include "basic_mechanical_integration_configuration.h"
#include "orvd/multibody_model/multibody_model.h"

namespace orvd::integrators::internal {

// Scalar validation is independent of the model and is performed before any
// dynamics evaluation. Even a family unused by the model must be valid.
void ValidateNewmarkConfiguration(const NewmarkConfiguration& configuration);

// Immutable coordinate semantics derived from finalized model handles. The
// owner has already verified that nz is the system's declared series-force
// range; this layout never infers physics from names or vector dimensions.
class NewmarkCoordinateLayout final {
   public:
    NewmarkCoordinateLayout(const multibody_model::MultibodyModel& model,
                            int series_force_state_size);

    [[nodiscard]] NewmarkCoreConfiguration Expand(
        const NewmarkConfiguration& configuration,
        const Eigen::Ref<const Eigen::VectorXd>& reference_q) const;

   private:
    enum class Family { kTranslation, kAngle, kQuaternion };
    struct Block {
        int start;
        int size;
        Family family;
    };
    int nq_;
    int nz_;
    std::vector<Block> blocks_;
};

}  // namespace orvd::integrators::internal
