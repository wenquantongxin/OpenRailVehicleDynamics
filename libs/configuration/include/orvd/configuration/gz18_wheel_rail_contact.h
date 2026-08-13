#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h"

namespace orvd::track_geometry {
class TrackGeometry;
}

namespace orvd::wheel_rail_contact {
class TrackIrregularityField;
}

namespace orvd::configuration {

class AssembledVehicleContactScenario;
struct VehicleDefinition;
namespace internal {
struct BoundWheelRailContact;
}

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
// prepared surfaces. The identifiers, pose constants, gauge data and inspected
// contact configuration do remain here because they are part of the binding,
// not transient loader state.
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
   private:
    friend class AssembledVehicleContactScenario;
    friend std::unique_ptr<Gz18WheelRailContact> AssembleGz18WheelRailContact(
        const std::filesystem::path&, const StartupWheelRailBinding&, double);
    friend std::unique_ptr<AssembledVehicleContactScenario>
    AssembleGz18ContactScenario(
        const VehicleDefinition&, const ResolvedStartupState&,
        track_geometry::TrackGeometry, const std::filesystem::path&, double,
        double,
        std::unique_ptr<wheel_rail_contact::TrackIrregularityField>);

    explicit Gz18WheelRailContact(
        std::unique_ptr<internal::BoundWheelRailContact> implementation);

    [[nodiscard]] std::unique_ptr<
        wheel_rail_contact::WheelRailContactRuntimePersonality>
    ReleaseRuntimePersonality();

    std::unique_ptr<internal::BoundWheelRailContact> implementation_;
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
