#include "orvd/configuration/irw_wheel_rail_contact.h"

#include <memory>
#include <utility>

#include "orvd/configuration/load_vehicle_contact_record.h"
#include "wheel_rail_contact_personality_assembly.h"

namespace orvd::configuration {
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

std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality>
IrwWheelRailContact::ReleaseRuntimePersonality() {
    return std::move(implementation_->runtime_personality);
}

std::unique_ptr<IrwWheelRailContact> AssembleIrwWheelRailContact(
    const std::filesystem::path& orvd_data_root, const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters) {
    return std::unique_ptr<IrwWheelRailContact>(
        new IrwWheelRailContact(internal::AssembleBoundWheelRailContact(
            orvd_data_root, startup_binding, rail_profile_reference_vertical_offset_meters,
            LoadVehicleContactRecordFromJsonFile(
                orvd_data_root / internal::kIrwContactRecordRelativePath))));
}

}  // namespace orvd::configuration
