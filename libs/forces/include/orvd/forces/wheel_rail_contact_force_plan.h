#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/track_geometry/track_geometry.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h"

// A frozen, model-bound wheel-rail force source.
//
// The plan owns the line and the immutable runtime contact personality.  The
// only object it borrows is the finalized multibody model it was compiled
// against.  Construction resolves every name and fixes every interface; a
// right-hand-side evaluation performs no lookup and changes no plan state.

namespace orvd::forces {

struct WheelRailContactCarrierDefinition {
    std::string carrier_name;
    std::string body_name;
    double initial_projection_station_meters{0.0};
};

struct WheelRailContactInterfaceDefinition {
    std::string interface_name;
    std::string carrier_name;
    std::string wheel_body_name;
    wheel_rail_contact::WheelSide side{
        wheel_rail_contact::WheelSide::kRight};
};

// Migration-time observations emitted from the same real contact evaluation
// that produced an interface wrench. They are not a second force law. Q and N
// are meaningful for zero or more patches; the scalar Tx/Ty pair is currently
// qualified only when one patch supplies its unique contact frame.
struct WheelRailContactInterfaceObservation {
    std::size_t contact_patch_count{0};
    double vertical_support_force_on_wheel_newtons{0.0};
    double normal_force_newtons{0.0};
    double longitudinal_force_on_wheel_newtons{0.0};
    double lateral_force_on_wheel_newtons{0.0};
};

class WheelRailContactForcePlan;

// Mutable scratch for one runtime context.  A serial evaluation reuses one
// heavy contact-core workspace across every interface.  The small carrier and
// pending-wrench arrays are sized once with the frozen plan.
class WheelRailContactForceWorkspace {
   public:
    ~WheelRailContactForceWorkspace();

    WheelRailContactForceWorkspace(const WheelRailContactForceWorkspace&) =
        delete;
    WheelRailContactForceWorkspace& operator=(
        const WheelRailContactForceWorkspace&) = delete;
    WheelRailContactForceWorkspace(WheelRailContactForceWorkspace&&) = delete;
    WheelRailContactForceWorkspace& operator=(
        WheelRailContactForceWorkspace&&) = delete;

   private:
    friend class WheelRailContactForcePlan;

    struct CarrierScratch {
        Eigen::Vector3d body_origin_in_inertial_meters{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation_inertial_from_body{
            Eigen::Matrix3d::Identity()};
        Eigen::Vector3d body_origin_velocity_in_inertial_meters_per_second{
            Eigen::Vector3d::Zero()};
        Eigen::Vector3d body_angular_velocity_in_inertial_radians_per_second{
            Eigen::Vector3d::Zero()};
        Eigen::Vector3d track_origin_in_inertial_meters{
            Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation_inertial_from_track{
            Eigen::Matrix3d::Identity()};
        Eigen::Vector3d body_origin_in_track_meters{Eigen::Vector3d::Zero()};
        Eigen::Vector3d body_origin_velocity_in_track_meters_per_second{
            Eigen::Vector3d::Zero()};
        Eigen::Vector3d body_angular_velocity_in_track_radians_per_second{
            Eigen::Vector3d::Zero()};
        double track_station_meters{0.0};
        double station_rate_meters_per_second{0.0};
        double path_rate_meters_per_second{0.0};
        double roll_radians{0.0};
        double yaw_radians{0.0};
        double wheel_pitch_rate_radians_per_second{0.0};
        double curvature_radians_per_meter{0.0};
    };

    WheelRailContactForceWorkspace(
        const WheelRailContactForcePlan* issuer, std::size_t carrier_count,
        std::size_t interface_count);

    const WheelRailContactForcePlan* issuer_;
    wheel_rail_contact::WheelRailContactWorkspace contact_workspace_;
    std::vector<CarrierScratch> carriers_;
    std::vector<multibody_model::AppliedBodyWrench> pending_wrenches_;
    std::vector<WheelRailContactInterfaceObservation>
        pending_interface_observations_;
};

class WheelRailContactForcePlan {
   public:
    WheelRailContactForcePlan(
        const multibody_model::MultibodyModel& model,
        track_geometry::TrackGeometry line,
        std::unique_ptr<
            wheel_rail_contact::WheelRailContactRuntimePersonality>
            personality,
        std::vector<WheelRailContactCarrierDefinition> carriers,
        std::vector<WheelRailContactInterfaceDefinition> interfaces,
        double projection_search_half_width_meters);
    WheelRailContactForcePlan(
        multibody_model::MultibodyModel&&, track_geometry::TrackGeometry,
        std::unique_ptr<
            wheel_rail_contact::WheelRailContactRuntimePersonality>,
        std::vector<WheelRailContactCarrierDefinition>,
        std::vector<WheelRailContactInterfaceDefinition>, double) = delete;
    WheelRailContactForcePlan(const WheelRailContactForcePlan&) = delete;
    WheelRailContactForcePlan& operator=(const WheelRailContactForcePlan&) =
        delete;
    WheelRailContactForcePlan(WheelRailContactForcePlan&&) = delete;
    WheelRailContactForcePlan& operator=(WheelRailContactForcePlan&&) = delete;

    [[nodiscard]] const multibody_model::MultibodyModel& model() const {
        return *model_;
    }
    [[nodiscard]] const track_geometry::TrackGeometry& track_geometry() const {
        return line_;
    }
    [[nodiscard]] int carrier_count() const {
        return static_cast<int>(carriers_.size());
    }
    [[nodiscard]] int interface_count() const {
        return static_cast<int>(interfaces_.size());
    }
    [[nodiscard]] int body_wrench_count() const { return interface_count(); }
    [[nodiscard]] std::string_view carrier_name(int index) const;
    [[nodiscard]] std::string_view interface_name(int index) const;
    [[nodiscard]] double initial_projection_station_meters(int index) const;

    [[nodiscard]] std::unique_ptr<WheelRailContactForceWorkspace>
    CreateWorkspace() const;

    // Evaluates every frozen interface once.  Every valid call writes exactly
    // `body_wrench_count()` entries, one equivalent wrench at each wheel-body
    // origin.  A no-contact interface writes a valid zero wrench rather than
    // disappearing from the topology.
    void CalcAppliedForces(
        const multibody_model::MultibodyEvaluationContext& context,
        WheelRailContactForceWorkspace& workspace,
        std::span<const double> projection_station_hints_meters,
        std::span<multibody_model::AppliedBodyWrench> body_wrenches) const;

    // Evaluates the same force path while also publishing its per-interface
    // migration observations. Zero- and one-patch interfaces are admitted;
    // more than one patch is refused because summing Tx/Ty scalars expressed
    // in distinct contact frames would have no defined meaning. The ordinary
    // force-only path continues to admit and combine multiple patches. Every
    // output span is replaced only after all interfaces succeed.
    void CalcAppliedForcesAndObservations(
        const multibody_model::MultibodyEvaluationContext& context,
        WheelRailContactForceWorkspace& workspace,
        std::span<const double> projection_station_hints_meters,
        std::span<multibody_model::AppliedBodyWrench> body_wrenches,
        std::span<WheelRailContactInterfaceObservation>
            interface_observations) const;

    // Re-evaluates only the unique carrier projections at one accepted
    // endpoint. The current hints select the local branches; the output is
    // written only after every carrier succeeds. No wheel contact kernel is
    // evaluated on this path.
    void CalcProjectionStationHints(
        const multibody_model::MultibodyEvaluationContext& context,
        WheelRailContactForceWorkspace& workspace,
        std::span<const double> current_hints_meters,
        std::span<double> updated_hints_meters) const;

    // G53's admitted topology has one GZ18 wheelset body serving as both the
    // shared projection carrier and the body receiving its left/right contact
    // wrenches.  Construction refuses split carrier/wheel bodies rather than
    // pretending to implement the different orientation and spin kinematics
    // required by an independently rotating wheel.  That topology is added
    // with IRW's real consumer, not guessed here.

   private:
    struct CarrierBinding {
        std::string name;
        multibody_model::RigidBodyHandle body;
        double initial_projection_station_meters{0.0};
    };
    struct InterfaceBinding {
        std::string name;
        std::size_t carrier_ordinal{0};
        multibody_model::RigidBodyHandle wheel_body;
        wheel_rail_contact::WheelSide side{
            wheel_rail_contact::WheelSide::kRight};
    };

    void EvaluateCarrierProjections(
        const multibody_model::MultibodyEvaluationContext& context,
        WheelRailContactForceWorkspace& workspace,
        std::span<const double> projection_station_hints_meters) const;
    void CompleteCarrierKinematics(
        const multibody_model::MultibodyEvaluationContext& context,
        WheelRailContactForceWorkspace& workspace) const;
    void CalcAppliedForcesImpl(
        const multibody_model::MultibodyEvaluationContext& context,
        WheelRailContactForceWorkspace& workspace,
        std::span<const double> projection_station_hints_meters,
        std::span<multibody_model::AppliedBodyWrench> body_wrenches,
        std::span<WheelRailContactInterfaceObservation> interface_observations,
        bool publish_observations) const;

    const multibody_model::MultibodyModel* model_;
    track_geometry::TrackGeometry line_;
    std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality>
        personality_;
    std::vector<CarrierBinding> carriers_;
    std::vector<InterfaceBinding> interfaces_;
    double projection_search_half_width_meters_{0.0};
};

}  // namespace orvd::forces
