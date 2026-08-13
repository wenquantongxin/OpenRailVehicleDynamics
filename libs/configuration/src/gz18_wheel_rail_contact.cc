#include "orvd/configuration/gz18_wheel_rail_contact.h"

#include <memory>
#include <utility>

#include "wheel_rail_contact_personality_assembly.h"

namespace orvd::configuration {
namespace {

using wheel_rail_contact::ContactGeometryConfiguration;
using wheel_rail_contact::ContactMaterial;
using wheel_rail_contact::CreepageConfiguration;
using wheel_rail_contact::FrictionLaw;
using wheel_rail_contact::NormalContactConfiguration;
using wheel_rail_contact::OutsideTableRule;
using wheel_rail_contact::RailProfileOriginMode;
using wheel_rail_contact::TangentialContactConfiguration;
using wheel_rail_contact::WheelRailContactConfiguration;

constexpr double kNominalRollingRadiusMeters = 0.42;
constexpr double kWheelLateralDatumMagnitudeMeters = 0.7465;
constexpr double kTrackGaugeMeters = 1.435;
constexpr double kGaugeMeasuringDepthMeters = 0.016;
// The pose path inherited the exact 1:40 angle. The geometry/force path was
// qualified with the rounded literal below in MakeGz18ContactConfiguration.
// They are two active source values and are deliberately not normalised here.
constexpr double kPoseRailCantRadians = 0.02499479361892016;

constexpr std::string_view kWheelProfileRelativePath =
    "vehicle_library/gz18/wheel_profiles/gz18_reference_wheel_profile.json";
constexpr std::string_view kRailProfileRelativePath =
    "track_library/rail_profiles/uic60_rail_profile.json";

WheelRailContactConfiguration MakeGz18ContactConfiguration() {
    const ContactMaterial material{
        .youngs_modulus_pascals = 210000000000.0,
        .poisson_ratio = 0.28,
    };
    const ContactGeometryConfiguration geometry{
        .nominal_rolling_radius_meters = kNominalRollingRadiusMeters,
        .envelope_bin_width_meters = 2.0e-5,
        .outline_sample_count = 1000,
        .contact_gap_epsilon_meters = 0.0,
        .island_merge_gap_tolerance_meters = 1.0e-6,
        .island_quadrature_stations = 120,
        .rail_cant_radians = 0.024994792,
    };
    const NormalContactConfiguration normal{
        .material = material,
        .penetration_equivalence_factor =
            internal::kEecPenetrationEquivalenceFactor,
        .reference_damping_newton_seconds_per_meter = 100000.0,
        .reference_stiffness_newtons_per_meter = 500000000.0,
        .damping_activation_penetration_meters = 1.0e-12,
    };
    const CreepageConfiguration creepage{
        .minimum_reference_speed_meters_per_second = 0.01,
    };
    const FrictionLaw friction{
        .static_coefficient = 0.3,
        .limiting_fraction = 0.4,
        .decay_per_meter_per_second = 0.55,
        .minimum_reference_speed_meters_per_second = 0.01,
    };
    const TangentialContactConfiguration tangential{
        .longitudinal_cells = 21,
        .lateral_strips = 21,
        .refinement_width = 0.01,
        .material = material,
    };
    return WheelRailContactConfiguration{
        .geometry = geometry,
        .normal = normal,
        .creepage = creepage,
        .friction = friction,
        .tangential = tangential,
        .material = material,
        .creep_coefficients_outside_table = OutsideTableRule::kAsymptotic,
    };
}

internal::WheelRailContactPersonalitySpecification MakeGz18Specification() {
    return internal::WheelRailContactPersonalitySpecification{
        .diagnostic_vehicle_name = "GZ18",
        .wheel_profile_identifier = kGz18WheelProfileIdentifier,
        .rail_profile_identifier = kGz18RailProfileIdentifier,
        .contact_strategy_identifier =
            kGz18WheelRailContactStrategyIdentifier,
        .wheel_profile_relative_path = kWheelProfileRelativePath,
        .rail_profile_relative_path = kRailProfileRelativePath,
        .contact_configuration = MakeGz18ContactConfiguration(),
        .track_gauge_meters = kTrackGaugeMeters,
        .gauge_measuring_depth_meters = kGaugeMeasuringDepthMeters,
        .pose_rail_cant_radians = kPoseRailCantRadians,
        .wheel_lateral_datum_magnitude_meters =
            kWheelLateralDatumMagnitudeMeters,
        .rail_profile_origin_mode = RailProfileOriginMode::kProfileCoordinate,
    };
}

}  // namespace

Gz18WheelRailContact::Gz18WheelRailContact(
    std::unique_ptr<internal::BoundWheelRailContact> implementation)
    : implementation_(std::move(implementation)) {}

Gz18WheelRailContact::~Gz18WheelRailContact() = default;

const StartupWheelRailBinding& Gz18WheelRailContact::binding() const {
    return implementation_->binding;
}

const wheel_rail_contact::WheelRailContactConfiguration&
Gz18WheelRailContact::contact_configuration() const {
    return implementation_->contact_configuration;
}

const wheel_rail_contact::WheelProfilePreprocessingConfiguration&
Gz18WheelRailContact::wheel_profile_preprocessing_configuration() const {
    return implementation_->wheel_profile_preprocessing_configuration;
}

double Gz18WheelRailContact::track_gauge_meters() const {
    return implementation_->track_gauge_meters;
}

double Gz18WheelRailContact::gauge_measuring_depth_meters() const {
    return implementation_->gauge_measuring_depth_meters;
}

double Gz18WheelRailContact::pose_rail_cant_radians() const {
    return implementation_->pose_rail_cant_radians;
}

wheel_rail_contact::RailProfileOriginMode
Gz18WheelRailContact::rail_profile_origin_mode() const {
    return implementation_->runtime_personality->rail_profile_origin_mode();
}

const wheel_rail_contact::RailGaugeDatum&
Gz18WheelRailContact::rail_gauge_datum(
    wheel_rail_contact::WheelSide side) const {
    return side == wheel_rail_contact::WheelSide::kRight
               ? implementation_->right_rail_gauge_datum
               : implementation_->left_rail_gauge_datum;
}

const wheel_rail_contact::WheelRailPoseConstants&
Gz18WheelRailContact::pose_constants(
    wheel_rail_contact::WheelSide side) const {
    return implementation_->runtime_personality->pose_constants(side);
}

const wheel_rail_contact::WheelRailContactModel& Gz18WheelRailContact::model(
    wheel_rail_contact::WheelSide side) const {
    return implementation_->runtime_personality->model(side);
}

std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality>
Gz18WheelRailContact::ReleaseRuntimePersonality() {
    return std::move(implementation_->runtime_personality);
}

std::unique_ptr<Gz18WheelRailContact> AssembleGz18WheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters) {
    return std::unique_ptr<Gz18WheelRailContact>(new Gz18WheelRailContact(
        internal::AssembleBoundWheelRailContact(
            orvd_data_root, startup_binding,
            rail_profile_reference_vertical_offset_meters,
            MakeGz18Specification())));
}

}  // namespace orvd::configuration
