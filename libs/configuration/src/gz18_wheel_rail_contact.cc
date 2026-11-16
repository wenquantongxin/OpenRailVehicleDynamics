#include "orvd/configuration/gz18_wheel_rail_contact.h"

#include <memory>
#include <utility>

#include "orvd/configuration/load_vehicle_contact_record.h"
#include "wheel_rail_contact_personality_assembly.h"

namespace orvd::configuration {
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
    const std::filesystem::path& orvd_data_root, const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters) {
    return std::unique_ptr<Gz18WheelRailContact>(
        new Gz18WheelRailContact(internal::AssembleBoundWheelRailContact(
            orvd_data_root, startup_binding, rail_profile_reference_vertical_offset_meters,
            LoadVehicleContactRecordFromJsonFile(
                orvd_data_root / internal::kGz18ContactRecordRelativePath))));
}

}  // namespace orvd::configuration
