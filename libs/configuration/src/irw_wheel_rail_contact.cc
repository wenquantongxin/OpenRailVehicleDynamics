#include "orvd/configuration/irw_wheel_rail_contact.h"

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
using wheel_rail_contact::ProfileTrackRollTransportPolicy;
using wheel_rail_contact::RailProfileOriginMode;
using wheel_rail_contact::TangentialContactConfiguration;
using wheel_rail_contact::WheelProfilePreprocessingConfiguration;
using wheel_rail_contact::WheelRailContactConfiguration;

constexpr double kNominalRollingRadiusMeters = 0.43;
constexpr double kWheelLateralDatumMagnitudeMeters = 0.7465;
constexpr double kTrackGaugeMeters = 1.435;
constexpr double kGaugeMeasuringDepthMeters = 0.016;
constexpr double kPoseRailCantRadians = 0.02499479361892016;

constexpr std::string_view kWheelProfileRelativePath =
    "vehicle_library/irw/wheel_profiles/irw_reference_wheel_profile.json";
constexpr std::string_view kRailProfileRelativePath =
    "track_library/rail_profiles/uic60_rail_profile.json";

WheelRailContactConfiguration MakeIrwContactConfiguration() {
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

internal::WheelRailContactPersonalitySpecification MakeIrwSpecification() {
    return internal::WheelRailContactPersonalitySpecification{
        .diagnostic_vehicle_name = "IRW",
        .wheel_profile_identifier = kIrwWheelProfileIdentifier,
        .rail_profile_identifier = kIrwRailProfileIdentifier,
        .contact_strategy_identifier =
            kIrwWheelRailContactStrategyIdentifier,
        .wheel_profile_relative_path = kWheelProfileRelativePath,
        .rail_profile_relative_path = kRailProfileRelativePath,
        .contact_configuration = MakeIrwContactConfiguration(),
        .wheel_profile_preprocessing_configuration =
            WheelProfilePreprocessingConfiguration{
                .equal_arc_length_rescan_step_meters = 0.0,
                .source_lateral_rediscretisation_step_meters = 0.0001,
            },
        .track_gauge_meters = kTrackGaugeMeters,
        .gauge_measuring_depth_meters = kGaugeMeasuringDepthMeters,
        .pose_rail_cant_radians = kPoseRailCantRadians,
        .wheel_lateral_datum_magnitude_meters =
            kWheelLateralDatumMagnitudeMeters,
        .rail_profile_origin_mode = RailProfileOriginMode::kProfileCoordinate,
        .roll_transport_policy = ProfileTrackRollTransportPolicy::kApplied,
    };
}

}  // namespace

IrwWheelRailContact::IrwWheelRailContact(
    std::unique_ptr<internal::BoundWheelRailContact> implementation)
    : implementation_(std::move(implementation)) {}

IrwWheelRailContact::~IrwWheelRailContact() = default;

const StartupWheelRailBinding& IrwWheelRailContact::binding() const {
    return implementation_->binding;
}

const wheel_rail_contact::WheelRailContactConfiguration&
IrwWheelRailContact::contact_configuration() const {
    return implementation_->contact_configuration;
}

const wheel_rail_contact::WheelProfilePreprocessingConfiguration&
IrwWheelRailContact::wheel_profile_preprocessing_configuration() const {
    return implementation_->wheel_profile_preprocessing_configuration;
}

double IrwWheelRailContact::track_gauge_meters() const {
    return implementation_->track_gauge_meters;
}

double IrwWheelRailContact::gauge_measuring_depth_meters() const {
    return implementation_->gauge_measuring_depth_meters;
}

double IrwWheelRailContact::pose_rail_cant_radians() const {
    return implementation_->pose_rail_cant_radians;
}

wheel_rail_contact::RailProfileOriginMode
IrwWheelRailContact::rail_profile_origin_mode() const {
    return implementation_->runtime_personality->rail_profile_origin_mode();
}

const wheel_rail_contact::RailGaugeDatum&
IrwWheelRailContact::rail_gauge_datum(
    wheel_rail_contact::WheelSide side) const {
    return side == wheel_rail_contact::WheelSide::kRight
               ? implementation_->right_rail_gauge_datum
               : implementation_->left_rail_gauge_datum;
}

const wheel_rail_contact::WheelRailPoseConstants&
IrwWheelRailContact::pose_constants(
    wheel_rail_contact::WheelSide side) const {
    return implementation_->runtime_personality->pose_constants(side);
}

const wheel_rail_contact::WheelRailContactModel& IrwWheelRailContact::model(
    wheel_rail_contact::WheelSide side) const {
    return implementation_->runtime_personality->model(side);
}

const wheel_rail_contact::ProfileTrackRollTransportStrategy&
IrwWheelRailContact::profile_track_roll_transport_strategy() const {
    return implementation_->runtime_personality->roll_transport_strategy();
}

std::unique_ptr<IrwWheelRailContact> AssembleIrwWheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters) {
    return std::unique_ptr<IrwWheelRailContact>(new IrwWheelRailContact(
        internal::AssembleBoundWheelRailContact(
            orvd_data_root, startup_binding,
            rail_profile_reference_vertical_offset_meters,
            MakeIrwSpecification())));
}

}  // namespace orvd::configuration
