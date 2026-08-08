#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"

namespace orvd::configuration {

inline constexpr std::string_view kGz18WheelProfileIdentifier =
    "gz18_reference_wheel_profile";
inline constexpr std::string_view kGz18RailProfileIdentifier =
    "uic60_rail_profile";
inline constexpr std::string_view kGz18WheelRailContactStrategyIdentifier =
    "gz18_reference_wheel_rail_contact";

// The installed GZ18 contact personality, resolved against one start-up
// binding.
//
// This object owns both side-specific immutable contact models and every fixed
// value the later vehicle/track connection needs beside them. The two loaded
// point lists need not remain alive: each model has already built and owns its
// prepared surfaces. The identifiers, pose constants and low-level roll policy
// do remain here because they are part of the binding, not transient loader
// state.
//
// The public configuration accessors are for scientific inspection. They
// return const references; changing a configuration after its model was built
// would create two authorities and is deliberately not supported.
class Gz18WheelRailContact {
   public:
    ~Gz18WheelRailContact();

    Gz18WheelRailContact(const Gz18WheelRailContact&) = delete;
    Gz18WheelRailContact& operator=(const Gz18WheelRailContact&) = delete;
    Gz18WheelRailContact(Gz18WheelRailContact&&) = delete;
    Gz18WheelRailContact& operator=(Gz18WheelRailContact&&) = delete;

    [[nodiscard]] const StartupWheelRailBinding& binding() const {
        return binding_;
    }
    [[nodiscard]] const wheel_rail_contact::WheelRailContactConfiguration&
    contact_configuration() const {
        return contact_configuration_;
    }
    [[nodiscard]]
    const wheel_rail_contact::WheelProfilePreprocessingConfiguration&
    wheel_profile_preprocessing_configuration() const {
        return wheel_profile_preprocessing_configuration_;
    }
    [[nodiscard]] double track_gauge_meters() const {
        return track_gauge_meters_;
    }
    [[nodiscard]] double gauge_measuring_depth_meters() const {
        return gauge_measuring_depth_meters_;
    }
    [[nodiscard]] double pose_rail_cant_radians() const {
        return pose_rail_cant_radians_;
    }

    [[nodiscard]] const wheel_rail_contact::RailGaugeDatum& rail_gauge_datum(
        wheel_rail_contact::WheelSide side) const {
        return side == wheel_rail_contact::WheelSide::kRight
                   ? right_rail_gauge_datum_
                   : left_rail_gauge_datum_;
    }
    [[nodiscard]] const wheel_rail_contact::WheelRailPoseConstants&
    pose_constants(wheel_rail_contact::WheelSide side) const {
        return side == wheel_rail_contact::WheelSide::kRight
                   ? right_pose_constants_
                   : left_pose_constants_;
    }
    [[nodiscard]] const wheel_rail_contact::WheelRailContactModel& model(
        wheel_rail_contact::WheelSide side) const {
        return side == wheel_rail_contact::WheelSide::kRight ? *right_model_
                                                             : *left_model_;
    }
    [[nodiscard]]
    const wheel_rail_contact::ProfileTrackRollTransportStrategy&
    profile_track_roll_transport_strategy() const {
        return profile_track_roll_transport_strategy_;
    }

   private:
    friend std::unique_ptr<Gz18WheelRailContact> AssembleGz18WheelRailContact(
        const std::filesystem::path&, const StartupWheelRailBinding&, double);

    Gz18WheelRailContact(
        StartupWheelRailBinding binding,
        wheel_rail_contact::WheelRailContactConfiguration contact_configuration,
        wheel_rail_contact::WheelProfilePreprocessingConfiguration
            wheel_profile_preprocessing_configuration,
        double track_gauge_meters, double gauge_measuring_depth_meters,
        double pose_rail_cant_radians,
        wheel_rail_contact::RailGaugeDatum right_rail_gauge_datum,
        wheel_rail_contact::RailGaugeDatum left_rail_gauge_datum,
        wheel_rail_contact::WheelRailPoseConstants right_pose_constants,
        wheel_rail_contact::WheelRailPoseConstants left_pose_constants,
        wheel_rail_contact::ProfileTrackRollTransportStrategy
            profile_track_roll_transport_strategy,
        std::unique_ptr<wheel_rail_contact::WheelRailContactModel> right_model,
        std::unique_ptr<wheel_rail_contact::WheelRailContactModel> left_model);

    StartupWheelRailBinding binding_;
    wheel_rail_contact::WheelRailContactConfiguration contact_configuration_;
    wheel_rail_contact::WheelProfilePreprocessingConfiguration
        wheel_profile_preprocessing_configuration_;
    double track_gauge_meters_{0.0};
    double gauge_measuring_depth_meters_{0.0};
    double pose_rail_cant_radians_{0.0};
    wheel_rail_contact::RailGaugeDatum right_rail_gauge_datum_;
    wheel_rail_contact::RailGaugeDatum left_rail_gauge_datum_;
    wheel_rail_contact::WheelRailPoseConstants right_pose_constants_;
    wheel_rail_contact::WheelRailPoseConstants left_pose_constants_;
    wheel_rail_contact::ProfileTrackRollTransportStrategy
        profile_track_roll_transport_strategy_;
    std::unique_ptr<wheel_rail_contact::WheelRailContactModel> right_model_;
    std::unique_ptr<wheel_rail_contact::WheelRailContactModel> left_model_;
};

// Loads the two GZ18 JSON profiles from an explicitly supplied ORVD data root,
// checks the start-up record's three logical identities against those assets,
// derives the two rail gauge datums and builds the immutable side models.
//
// `orvd_data_root` is the directory containing `vehicle_library` and
// `track_library`. It is never searched for, read from an environment variable
// or compiled into the library. The three binding strings are identities rather
// than paths; path-like spellings are refused before the closed GZ18 mapping is
// consulted.
//
// Throws std::runtime_error when an asset cannot be read. Throws
// std::invalid_argument when an identity, asset role, asset-internal identifier,
// vertical offset, profile or GZ18 configuration is unusable.
[[nodiscard]] std::unique_ptr<Gz18WheelRailContact>
AssembleGz18WheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters);

}  // namespace orvd::configuration
