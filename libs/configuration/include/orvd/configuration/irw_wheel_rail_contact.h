#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"

namespace orvd::configuration {

namespace internal {
struct BoundWheelRailContact;
}

inline constexpr std::string_view kIrwWheelProfileIdentifier =
    "irw_reference_wheel_profile";
inline constexpr std::string_view kIrwRailProfileIdentifier =
    "uic60_rail_profile";
inline constexpr std::string_view kIrwWheelRailContactStrategyIdentifier =
    "irw_reference_wheel_rail_contact";

// The installed IRW contact personality resolved against one start-up
// binding. It owns the prepared left/right S1002-UIC60 contact models and all
// fixed inputs needed by G69's later vehicle connection.
//
// G68 deliberately stops at this immutable vehicle personality. It does not
// infer the axle-bridge / independent-wheel kinematics or choose the eight
// wrench application bodies; those are topology bindings owned by G69.
class IrwWheelRailContact {
   public:
    ~IrwWheelRailContact();

    IrwWheelRailContact(const IrwWheelRailContact&) = delete;
    IrwWheelRailContact& operator=(const IrwWheelRailContact&) = delete;
    IrwWheelRailContact(IrwWheelRailContact&&) = delete;
    IrwWheelRailContact& operator=(IrwWheelRailContact&&) = delete;

    [[nodiscard]] const StartupWheelRailBinding& binding() const;
    [[nodiscard]] const wheel_rail_contact::WheelRailContactConfiguration&
    contact_configuration() const;
    [[nodiscard]]
    const wheel_rail_contact::WheelProfilePreprocessingConfiguration&
    wheel_profile_preprocessing_configuration() const;
    [[nodiscard]] double track_gauge_meters() const;
    [[nodiscard]] double gauge_measuring_depth_meters() const;
    [[nodiscard]] double pose_rail_cant_radians() const;
    [[nodiscard]] wheel_rail_contact::RailProfileOriginMode
    rail_profile_origin_mode() const;

    [[nodiscard]] const wheel_rail_contact::RailGaugeDatum& rail_gauge_datum(
        wheel_rail_contact::WheelSide side) const;
    [[nodiscard]] const wheel_rail_contact::WheelRailPoseConstants&
    pose_constants(wheel_rail_contact::WheelSide side) const;
    [[nodiscard]] const wheel_rail_contact::WheelRailContactModel& model(
        wheel_rail_contact::WheelSide side) const;
    [[nodiscard]]
    const wheel_rail_contact::ProfileTrackRollTransportStrategy&
    profile_track_roll_transport_strategy() const;

   private:
    friend std::unique_ptr<IrwWheelRailContact> AssembleIrwWheelRailContact(
        const std::filesystem::path&, const StartupWheelRailBinding&, double);

    explicit IrwWheelRailContact(
        std::unique_ptr<internal::BoundWheelRailContact> implementation);

    std::unique_ptr<internal::BoundWheelRailContact> implementation_;
};

// Loads the IRW S1002 wheel and shared UIC60 rail JSON profiles from an
// explicitly supplied ORVD data root, closes the three logical identities and
// builds the immutable side models. The identifiers are not paths and the
// function never searches the process environment for assets.
//
// Throws std::runtime_error when an asset cannot be read. Throws
// std::invalid_argument when a binding, asset or typed IRW contact input is
// unusable.
[[nodiscard]] std::unique_ptr<IrwWheelRailContact>
AssembleIrwWheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters);

}  // namespace orvd::configuration
