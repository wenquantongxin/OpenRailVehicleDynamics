#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"
#include "vehicle_project_element_property_tables.h"

// What every member of every settings record is: the settings' counterpart of
// the element tables (vehicle_project_element_property_tables.h), and the one
// place that knows.
//
// For every ProjectSettings, SettingsRecords<settings> states the record the
// settings are, where the project keeps it, the word the modelling service
// writes the settings as and how a diagnostic names them. Each of those
// records has a table listing its members in declaration order, a member of a
// record it holds in that record's place, each under its own name, prefixed
// for a held record's member with the holding member's name and a dot. A
// member is read-only or editable, and states what it holds by its type: a
// number, a number absent until it is stated, a count, a text, a path read as
// text, or a token standing for an enumeration's value.
//
// Reading walks a table in its order; assigning finds one property in it.

namespace orvd::vehicle_project_editing::internal {

// How a rail profile's origin is placed, and what the contact computation
// does outside the creep coefficient table: the words the contact record
// file writes them as (libs/configuration's contact record JSON), which the
// kernel exports no list of.
inline constexpr ValueTokens<wheel_rail_contact::RailProfileOriginMode, 2>
    kRailProfileOriginModeTokens{
        .tokens = {"track_station", "profile_coordinate"},
        .values = {wheel_rail_contact::RailProfileOriginMode::kTrackStation,
                   wheel_rail_contact::RailProfileOriginMode::kProfileCoordinate}};

inline constexpr ValueTokens<wheel_rail_contact::OutsideTableRule, 2> kOutsideTableRuleTokens{
    .tokens = {"refuse", "asymptotic"},
    .values = {wheel_rail_contact::OutsideTableRule::kRefuse,
               wheel_rail_contact::OutsideTableRule::kAsymptotic}};

// A member of a settings record, or of a record it holds, reached from the
// settings record whether that is seen as const or as mutable.
template <typename Record, typename Value>
struct SettingsRecordMember {
    const Value& (*in_const)(const Record& record);
    Value& (*in_mutable)(Record& record);
};

// A member holding the value one of `tokens` stands for.
template <typename Record, typename Value>
struct SettingsTokenMember {
    SettingsRecordMember<Record, Value> member;
    std::span<const std::string_view> tokens;
    std::span<const Value> values;
};

// What a read-only member holds.
template <typename Record>
using ReadOnlySettingsMember =
    std::variant<SettingsRecordMember<Record, double>, SettingsRecordMember<Record, std::size_t>,
                 SettingsRecordMember<Record, std::string>,
                 SettingsRecordMember<Record, std::filesystem::path>,
                 SettingsTokenMember<Record, wheel_rail_contact::OutsideTableRule>>;

// What an editable member holds.
template <typename Record>
using EditableSettingsMember =
    std::variant<SettingsRecordMember<Record, double>,
                 SettingsRecordMember<Record, std::optional<double>>,
                 SettingsTokenMember<Record, wheel_rail_contact::RailProfileOriginMode>>;

// One property: a member of the settings record and its name.
template <typename Record>
struct SettingsRecordProperty {
    std::string_view name;
    std::variant<ReadOnlySettingsMember<Record>, EditableSettingsMember<Record>> member;

    [[nodiscard]] constexpr bool editable() const {
        return std::holds_alternative<EditableSettingsMember<Record>>(member);
    }
};

// The class whose member `MemberPointer` points to.
template <typename MemberPointer>
struct ClassOfMember;

template <typename Class, typename Member>
struct ClassOfMember<Member Class::*> {
    using Type = Class;
};

// The member reached from `record` by `first` and then by each of `rest`,
// sharing the record's constness.
template <auto first, auto... rest>
constexpr auto& ReachMember(auto& record) {
    if constexpr (sizeof...(rest) == 0) {
        return record.*first;
    } else {
        return ReachMember<rest...>(record.*first);
    }
}

// The member of a settings record that ReachMember reaches.
template <auto first, auto... rest>
constexpr auto SettingsMember() {
    using Record = typename ClassOfMember<decltype(first)>::Type;
    using Value =
        std::remove_cvref_t<decltype(ReachMember<first, rest...>(std::declval<Record&>()))>;
    return SettingsRecordMember<Record, Value>{
        [](const Record& record) -> const Value& { return ReachMember<first, rest...>(record); },
        [](Record& record) -> Value& { return ReachMember<first, rest...>(record); }};
}

template <typename Record, typename Value, std::size_t count>
constexpr SettingsTokenMember<Record, Value> SettingsToken(
    SettingsRecordMember<Record, Value> member, const ValueTokens<Value, count>& tokens) {
    return {member, tokens.tokens, tokens.values};
}

// Every member of each settings record, in declaration order.

constexpr auto PropertiesOfSettingsRecord(std::type_identity<vehicle_project::VehicleProject>) {
    using Record = vehicle_project::VehicleProject;
    return std::to_array<SettingsRecordProperty<Record>>({
        {"gravitational_acceleration_meters_per_second_squared",
         EditableSettingsMember<Record>(
             SettingsMember<&Record::gravitational_acceleration_meters_per_second_squared>())},
    });
}

constexpr auto PropertiesOfSettingsRecord(std::type_identity<configuration::VehicleDefinition>) {
    using Record = configuration::VehicleDefinition;
    return std::to_array<SettingsRecordProperty<Record>>({
        {"vehicle_name", ReadOnlySettingsMember<Record>(SettingsMember<&Record::vehicle_name>())},
        {"mechanical_definition_identifier",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::mechanical_definition_identifier>())},
    });
}

constexpr auto PropertiesOfSettingsRecord(
    std::type_identity<configuration::VehicleWheelContactConfiguration>) {
    using Record = configuration::VehicleWheelContactConfiguration;
    using Profile = configuration::ContactProfileReference;
    return std::to_array<SettingsRecordProperty<Record>>({
        {"profile.profile_identifier",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::profile, &Profile::profile_identifier>())},
        {"profile.profile_path", ReadOnlySettingsMember<Record>(
                                     SettingsMember<&Record::profile, &Profile::profile_path>())},
        {"nominal_rolling_radius_meters",
         EditableSettingsMember<Record>(SettingsMember<&Record::nominal_rolling_radius_meters>())},
        {"wheel_lateral_datum_magnitude_meters",
         EditableSettingsMember<Record>(
             SettingsMember<&Record::wheel_lateral_datum_magnitude_meters>())},
    });
}

constexpr auto PropertiesOfSettingsRecord(
    std::type_identity<configuration::VehicleRailContactConfiguration>) {
    using Record = configuration::VehicleRailContactConfiguration;
    using Profile = configuration::ContactProfileReference;
    return std::to_array<SettingsRecordProperty<Record>>({
        {"profile.profile_identifier",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::profile, &Profile::profile_identifier>())},
        {"profile.profile_path", ReadOnlySettingsMember<Record>(
                                     SettingsMember<&Record::profile, &Profile::profile_path>())},
        {"track_gauge_meters",
         EditableSettingsMember<Record>(SettingsMember<&Record::track_gauge_meters>())},
        {"gauge_measuring_depth_meters",
         EditableSettingsMember<Record>(SettingsMember<&Record::gauge_measuring_depth_meters>())},
        {"pose_rail_cant_radians",
         EditableSettingsMember<Record>(SettingsMember<&Record::pose_rail_cant_radians>())},
        {"geometry_rail_cant_radians",
         EditableSettingsMember<Record>(SettingsMember<&Record::geometry_rail_cant_radians>())},
        {"profile_origin_mode",
         EditableSettingsMember<Record>(SettingsToken(
             SettingsMember<&Record::profile_origin_mode>(), kRailProfileOriginModeTokens))},
    });
}

constexpr auto PropertiesOfSettingsRecord(
    std::type_identity<configuration::VehicleContactComputationSettings>) {
    using Record = configuration::VehicleContactComputationSettings;
    using Material = wheel_rail_contact::ContactMaterial;
    using Geometry = configuration::VehicleContactGeometrySettings;
    using Normal = configuration::VehicleNormalContactSettings;
    using Creepage = wheel_rail_contact::CreepageConfiguration;
    using Friction = wheel_rail_contact::FrictionLaw;
    using Tangential = configuration::VehicleTangentialContactSettings;
    using Preprocessing = wheel_rail_contact::WheelProfilePreprocessingConfiguration;
    return std::to_array<SettingsRecordProperty<Record>>({
        {"contact_strategy_identifier",
         ReadOnlySettingsMember<Record>(SettingsMember<&Record::contact_strategy_identifier>())},
        {"material.youngs_modulus_pascals",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::material, &Material::youngs_modulus_pascals>())},
        {"material.poisson_ratio",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::material, &Material::poisson_ratio>())},
        {"geometry.envelope_bin_width_meters",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::geometry, &Geometry::envelope_bin_width_meters>())},
        {"geometry.outline_sample_count",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::geometry, &Geometry::outline_sample_count>())},
        {"geometry.contact_gap_epsilon_meters",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::geometry, &Geometry::contact_gap_epsilon_meters>())},
        {"geometry.island_merge_gap_tolerance_meters",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::geometry, &Geometry::island_merge_gap_tolerance_meters>())},
        {"geometry.island_quadrature_stations",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::geometry, &Geometry::island_quadrature_stations>())},
        {"normal.penetration_equivalence_factor",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::normal, &Normal::penetration_equivalence_factor>())},
        {"normal.reference_damping_newton_seconds_per_meter",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::normal,
                            &Normal::reference_damping_newton_seconds_per_meter>())},
        {"normal.reference_stiffness_newtons_per_meter",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::normal, &Normal::reference_stiffness_newtons_per_meter>())},
        {"normal.damping_activation_penetration_meters",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::normal, &Normal::damping_activation_penetration_meters>())},
        {"creepage.minimum_reference_speed_meters_per_second",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::creepage,
                            &Creepage::minimum_reference_speed_meters_per_second>())},
        {"friction.static_coefficient",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::friction, &Friction::static_coefficient>())},
        {"friction.limiting_fraction",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::friction, &Friction::limiting_fraction>())},
        {"friction.decay_per_meter_per_second",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::friction, &Friction::decay_per_meter_per_second>())},
        {"friction.minimum_reference_speed_meters_per_second",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::friction,
                            &Friction::minimum_reference_speed_meters_per_second>())},
        {"tangential.longitudinal_cells",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::tangential, &Tangential::longitudinal_cells>())},
        {"tangential.lateral_strips",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::tangential, &Tangential::lateral_strips>())},
        {"tangential.refinement_width",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::tangential, &Tangential::refinement_width>())},
        {"creep_coefficients_outside_table",
         ReadOnlySettingsMember<Record>(
             SettingsToken(SettingsMember<&Record::creep_coefficients_outside_table>(),
                           kOutsideTableRuleTokens))},
        {"wheel_profile_preprocessing.equal_arc_length_rescan_step_meters",
         ReadOnlySettingsMember<Record>(
             SettingsMember<&Record::wheel_profile_preprocessing,
                            &Preprocessing::equal_arc_length_rescan_step_meters>())},
    });
}

template <typename Record>
inline constexpr auto kSettingsRecordProperties =
    PropertiesOfSettingsRecord(std::type_identity<Record>{});

template <ProjectSettings settings>
struct SettingsRecords;

// Each RecordIn() returns a pointer to the settings' record in the project,
// sharing its constness. It is never null for settings the project always
// holds; for the wheel, the rail and the contact computation it is null while
// the project has no contact record.

template <>
struct SettingsRecords<ProjectSettings::kGravity> {
    using Record = vehicle_project::VehicleProject;
    static constexpr std::string_view kSettingsWord = "gravity";
    static constexpr std::string_view kSettingsDescription = "gravity";
    static auto* RecordIn(vehicle_project::ConstOrMutableVehicleProject auto& project) {
        return &project;
    }
};

template <>
struct SettingsRecords<ProjectSettings::kVehicleIdentity> {
    using Record = configuration::VehicleDefinition;
    static constexpr std::string_view kSettingsWord = "vehicle_identity";
    static constexpr std::string_view kSettingsDescription = "vehicle identity";
    static auto* RecordIn(vehicle_project::ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition;
    }
};

template <>
struct SettingsRecords<ProjectSettings::kWheel> {
    using Record = configuration::VehicleWheelContactConfiguration;
    static constexpr std::string_view kSettingsWord = "wheel";
    static constexpr std::string_view kSettingsDescription = "wheel settings";
    static auto* RecordIn(vehicle_project::ConstOrMutableVehicleProject auto& project) {
        return project.contact_record ? &project.contact_record->wheel : nullptr;
    }
};

template <>
struct SettingsRecords<ProjectSettings::kRail> {
    using Record = configuration::VehicleRailContactConfiguration;
    static constexpr std::string_view kSettingsWord = "rail";
    static constexpr std::string_view kSettingsDescription = "rail settings";
    static auto* RecordIn(vehicle_project::ConstOrMutableVehicleProject auto& project) {
        return project.contact_record ? &project.contact_record->rail : nullptr;
    }
};

template <>
struct SettingsRecords<ProjectSettings::kContactComputation> {
    using Record = configuration::VehicleContactComputationSettings;
    static constexpr std::string_view kSettingsWord = "contact_computation";
    static constexpr std::string_view kSettingsDescription = "contact computation settings";
    static auto* RecordIn(vehicle_project::ConstOrMutableVehicleProject auto& project) {
        return project.contact_record ? &project.contact_record->computation : nullptr;
    }
};

// Calls `visitor` with SettingsRecords<settings>{} for the `settings` given at
// run time and returns what it returns, which must be one type for all.
//
// Throws std::invalid_argument when `settings` is none of ProjectSettings'
// enumerators, which only a cast can produce.
template <typename Visitor>
decltype(auto) VisitProjectSettings(ProjectSettings settings, Visitor&& visitor) {
    switch (settings) {
        case ProjectSettings::kGravity:
            return visitor(SettingsRecords<ProjectSettings::kGravity>{});
        case ProjectSettings::kVehicleIdentity:
            return visitor(SettingsRecords<ProjectSettings::kVehicleIdentity>{});
        case ProjectSettings::kWheel:
            return visitor(SettingsRecords<ProjectSettings::kWheel>{});
        case ProjectSettings::kRail:
            return visitor(SettingsRecords<ProjectSettings::kRail>{});
        case ProjectSettings::kContactComputation:
            return visitor(SettingsRecords<ProjectSettings::kContactComputation>{});
    }
    throw std::invalid_argument("vehicle project settings " +
                                std::to_string(std::to_underlying(settings)) +
                                " are none of the project's settings");
}

// How a diagnostic names settings: their kSettingsDescription, "wheel
// settings".
inline std::string_view DescribeProjectSettings(ProjectSettings settings) {
    return VisitProjectSettings(
        settings, []<typename Records>(Records) { return Records::kSettingsDescription; });
}

// Why the project holds no record for the settings, which only the wheel, the
// rail and the contact computation can lack, with the contact record.
inline std::string DescribeMissingSettingsRecord(ProjectSettings settings) {
    return "this project has no wheel-rail contact record yet, so it has no " +
           std::string(DescribeProjectSettings(settings));
}

}  // namespace orvd::vehicle_project_editing::internal
