#pragma once

#include <memory>

namespace orvd::multibody_model {

class MultibodyModel;

/// Model-bound, preallocated storage for one forward-dynamics call.
class ForwardDynamicsWorkspace {
   public:
    ~ForwardDynamicsWorkspace();

    ForwardDynamicsWorkspace(const ForwardDynamicsWorkspace&) = delete;
    ForwardDynamicsWorkspace& operator=(const ForwardDynamicsWorkspace&) =
        delete;
    ForwardDynamicsWorkspace(ForwardDynamicsWorkspace&&) = delete;
    ForwardDynamicsWorkspace& operator=(ForwardDynamicsWorkspace&&) = delete;

   private:
    friend class MultibodyModel;
    class Implementation;
    explicit ForwardDynamicsWorkspace(
        std::unique_ptr<Implementation> implementation);
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::multibody_model
