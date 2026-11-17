#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "vehicle_project/vehicle_project_comparison.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// The project's settings edited as an element's properties are, on GZ18 and
// IRW. The gravity is read, set and undone, and set where the project stated
// none; a gravity the assembly refuses is the check's to report, not the
// edit's. The wheel's nominal rolling radius reaches the preview's wheels, the
// rail's profile origin mode takes each of its choices and nothing else, and a
// number that is not finite is refused. Exactly the properties the contract
// leaves read-only are refused, each by name; the wheel, the rail and the
// contact computation are refused while there is no contact record. A
// settings edit is one undo step and an unsaved change, and what it wrote is
// read back after saving and reopening.

namespace {

using namespace orvd;
using editing_verification::Fail;
using editing_verification::PlacedProjectWithContact;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using vehicle_project::VehicleProject;
using vehicle_project::VehicleProjectFileLocation;
using vehicle_project_comparison::SameProject;
using vehicle_project_editing::ProjectSettings;
using vehicle_project_editing::PropertyReading;
using vehicle_project_editing::PropertySubject;
using vehicle_project_editing::PropertyText;
using vehicle_project_editing::PropertyToken;
using vehicle_project_editing::PropertyValue;
using vehicle_project_editing::PropertyValueKind;
using vehicle_project_editing::VehicleProjectEditingSession;

constexpr std::string_view kGravity = "gravitational_acceleration_meters_per_second_squared";

// The settings properties the contract makes editable, stated here
// independently of the library; every other settings property is read-only.
const std::set<std::pair<ProjectSettings, std::string>> kEditableSettingsProperties = {
    {ProjectSettings::kGravity, std::string(kGravity)},
    {ProjectSettings::kWheel, "nominal_rolling_radius_meters"},
    {ProjectSettings::kWheel, "wheel_lateral_datum_magnitude_meters"},
    {ProjectSettings::kRail, "track_gauge_meters"},
    {ProjectSettings::kRail, "gauge_measuring_depth_meters"},
    {ProjectSettings::kRail, "pose_rail_cant_radians"},
    {ProjectSettings::kRail, "geometry_rail_cant_radians"},
    {ProjectSettings::kRail, "profile_origin_mode"},
};

std::string Describe(ProjectSettings settings) {
    return "the " + std::string(vehicle_project_editing::ProjectSettingsWord(settings)) +
           " settings";
}

PropertyReading ReadingOf(const VehicleProject& project, ProjectSettings settings,
                          std::string_view property_name) {
    const std::vector<PropertyReading> readings =
        vehicle_project_editing::ReadSubjectProperties(project, settings);
    const auto reading =
        std::ranges::find_if(readings, [property_name](const PropertyReading& candidate) {
            return candidate.description.name == property_name;
        });
    Require(reading != readings.end(),
            Describe(settings) + " have no property '" + std::string(property_name) + "'");
    return *reading;
}

double NumberOf(const VehicleProject& project, ProjectSettings settings,
                std::string_view property_name) {
    const PropertyReading reading = ReadingOf(project, settings, property_name);
    Require(reading.value.has_value() && std::holds_alternative<double>(*reading.value),
            Describe(settings) + " do not read '" + std::string(property_name) + "' as a number");
    return std::get<double>(*reading.value);
}

std::string TokenOf(const VehicleProject& project, ProjectSettings settings,
                    std::string_view property_name) {
    const PropertyReading reading = ReadingOf(project, settings, property_name);
    Require(reading.value.has_value() && std::holds_alternative<PropertyToken>(*reading.value),
            Describe(settings) + " do not read '" + std::string(property_name) + "' as a token");
    return std::get<PropertyToken>(*reading.value).token;
}

// `edit` must be refused, naming `settings` and `property_name`, leaving the
// session as it was.
template <typename Edit>
void RequireRefused(VehicleProjectEditingSession& session, ProjectSettings settings,
                    std::string_view property_name, Edit edit, std::string_view reason,
                    const std::string& what) {
    const VehicleProject before = session.project();
    const std::uint64_t revision = session.revision();
    const bool could_undo = session.can_undo();
    const auto outcome = edit();
    Require(!outcome.has_value(), what + " was accepted");
    Require(outcome.error().subject == PropertySubject{settings} &&
                outcome.error().property_name == std::string(property_name) &&
                outcome.error().reason.find(reason) != std::string::npos,
            what + " was refused without naming " + Describe(settings) + ", '" +
                std::string(property_name) + "' and why: " + outcome.error().reason);
    Require(SameProject(before, session.project()) && session.revision() == revision &&
                session.can_undo() == could_undo,
            what + " changed the session");
}

void VerifyGravity(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const double stated = gz18.startup_state.gravitational_acceleration_meters_per_second_squared;
    const PropertyReading opened =
        ReadingOf(session.project(), ProjectSettings::kGravity, kGravity);
    Require(opened.description.value_kind == PropertyValueKind::kNumber &&
                opened.description.editable &&
                NumberOf(session.project(), ProjectSettings::kGravity, kGravity) == stated,
            "the gravity does not read as the start-up state's editable number");

    const auto set = session.SetSubjectProperty(ProjectSettings::kGravity, kGravity, 9.8);
    Require(set.has_value() && *set == PropertySubject{ProjectSettings::kGravity},
            "setting the gravity was refused or named another subject");
    Require(NumberOf(session.project(), ProjectSettings::kGravity, kGravity) == 9.8 &&
                session.project().gravitational_acceleration_meters_per_second_squared == 9.8 &&
                session.revision() == 1 && session.can_undo() && session.has_unsaved_changes(),
            "setting the gravity was not one edit with an unsaved change");
    session.Undo();
    Require(NumberOf(session.project(), ProjectSettings::kGravity, kGravity) == stated &&
                !session.can_undo() && !session.has_unsaved_changes(),
            "undoing the gravity edit did not bring the stated gravity back");

    // A gravity the assembly refuses is the check's to report.
    Require(session.CheckMechanics().refusals.empty(), "GZ18 did not pass the check");
    Require(session.SetSubjectProperty(ProjectSettings::kGravity, kGravity, -9.81).has_value(),
            "a negative gravity was refused by the edit, which must leave it to the check");
    Require(!session.CheckMechanics().refusals.empty(),
            "the check did not report a negative gravity");

    RequireRefused(
        session, ProjectSettings::kGravity, kGravity,
        [&session] {
            return session.SetSubjectProperty(ProjectSettings::kGravity, kGravity,
                                              std::numeric_limits<double>::quiet_NaN());
        },
        "finite", "a gravity that is not a number");

    // A project that states no gravity reads none, and takes one.
    VehicleProject without_gravity = PlacedProjectWithContact(gz18);
    without_gravity.gravitational_acceleration_meters_per_second_squared.reset();
    VehicleProjectEditingSession stating(without_gravity);
    const PropertyReading absent =
        ReadingOf(stating.project(), ProjectSettings::kGravity, kGravity);
    Require(!absent.value.has_value() && absent.description.editable,
            "a project stating no gravity does not read an absent, editable gravity");
    Require(stating.SetSubjectProperty(ProjectSettings::kGravity, kGravity, stated).has_value() &&
                stating.project().gravitational_acceleration_meters_per_second_squared == stated &&
                stating.CheckMechanics().refusals.empty(),
            "stating the gravity of a project that stated none did not let it pass the check");
}

void VerifyWheelRadiusReachesPreview(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    constexpr std::string_view kRadius = "nominal_rolling_radius_meters";
    const double radius = NumberOf(session.project(), ProjectSettings::kWheel, kRadius);
    Require(session.CheckMechanics().refusals.empty() && !session.preview()->wheels.empty() &&
                std::ranges::all_of(session.preview()->wheels,
                                    [radius](const auto& wheel) {
                                        return wheel.nominal_rolling_radius_meters == radius;
                                    }),
            "GZ18's wheels are not drawn at the wheel's nominal rolling radius");
    const double larger = radius * 1.1;
    Require(session.SetSubjectProperty(ProjectSettings::kWheel, kRadius, larger).has_value(),
            "setting the wheel's nominal rolling radius was refused");
    Require(session.CheckMechanics().refusals.empty() &&
                std::ranges::all_of(session.preview()->wheels,
                                    [larger](const auto& wheel) {
                                        return wheel.nominal_rolling_radius_meters == larger;
                                    }),
            "the preview's wheels did not take the new nominal rolling radius");
    RequireRefused(
        session, ProjectSettings::kWheel, kRadius,
        [&session, kRadius] {
            return session.SetSubjectProperty(ProjectSettings::kWheel, kRadius,
                                              std::numeric_limits<double>::infinity());
        },
        "finite", "an infinite nominal rolling radius");
}

void VerifyRailOriginMode(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    constexpr std::string_view kOriginMode = "profile_origin_mode";
    const PropertyReading reading =
        ReadingOf(session.project(), ProjectSettings::kRail, kOriginMode);
    // The words the contact record file writes the origin modes as.
    const std::vector<std::string_view> modes = {"track_station", "profile_coordinate"};
    Require(reading.description.value_kind == PropertyValueKind::kToken &&
                std::ranges::equal(reading.description.choices, modes),
            "the rail's profile origin mode is not a token of the file's words");
    for (const std::string_view mode : modes) {
        Require(
            session.SetSubjectProperty(ProjectSettings::kRail, kOriginMode,
                                       PropertyToken{std::string(mode)})
                    .has_value() &&
                TokenOf(session.project(), ProjectSettings::kRail, kOriginMode) == mode,
            "setting the rail's profile origin mode to '" + std::string(mode) + "' did not take");
    }
    RequireRefused(
        session, ProjectSettings::kRail, kOriginMode,
        [&session, kOriginMode] {
            return session.SetSubjectProperty(ProjectSettings::kRail, kOriginMode,
                                              PropertyToken{"sideways"});
        },
        "'track_station', 'profile_coordinate'", "a profile origin mode that is none");
    // The two cants say which they are for.
    Require(ReadingOf(session.project(), ProjectSettings::kRail, "pose_rail_cant_radians")
                    .description.editable &&
                ReadingOf(session.project(), ProjectSettings::kRail, "geometry_rail_cant_radians")
                    .description.editable,
            "the rail's two cants are not both editable");
}

// Exactly the properties the contract leaves read-only are refused, each by
// name; a count reads as a number and an enumeration as a token.
void VerifyReadOnlySettings(const ReferenceVehicle& reference) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(reference));
    std::set<std::pair<ProjectSettings, std::string>> editable;
    for (const ProjectSettings settings : vehicle_project_editing::kProjectSettings) {
        for (const PropertyReading& reading :
             vehicle_project_editing::ReadSubjectProperties(session.project(), settings)) {
            const std::string name(reading.description.name);
            const std::string what = Describe(settings) + " property '" + name + "'";
            Require(reading.description.value_kind != PropertyValueKind::kElementName,
                    what + " is described as an element's own name");
            if (reading.description.editable) {
                editable.emplace(settings, name);
                continue;
            }
            Require(reading.value.has_value(), what + " has no value");
            RequireRefused(
                session, settings, name,
                [&session, settings, &name, &reading] {
                    return session.SetSubjectProperty(settings, name, *reading.value);
                },
                "read-only", "writing " + what + " back");
        }
    }
    Require(editable == kEditableSettingsProperties,
            reference.definition.vehicle_name +
                ": the editable settings properties are not exactly the contract's");

    const PropertyReading count = ReadingOf(session.project(), ProjectSettings::kContactComputation,
                                            "geometry.outline_sample_count");
    const PropertyReading rule = ReadingOf(session.project(), ProjectSettings::kContactComputation,
                                           "creep_coefficients_outside_table");
    const PropertyReading identity =
        ReadingOf(session.project(), ProjectSettings::kVehicleIdentity, "vehicle_name");
    const PropertyReading reference_body =
        ReadingOf(session.project(), ProjectSettings::kStationLayout, "reference_body_name");
    Require(count.description.value_kind == PropertyValueKind::kNumber &&
                std::get<double>(*count.value) ==
                    static_cast<double>(
                        reference.contact_record.computation.geometry.outline_sample_count) &&
                rule.description.value_kind == PropertyValueKind::kToken &&
                identity.description.value_kind == PropertyValueKind::kText &&
                std::get<PropertyText>(*identity.value).text == reference.definition.vehicle_name &&
                reference_body.description.value_kind == PropertyValueKind::kElementReference,
            reference.definition.vehicle_name +
                ": a count, an enumeration, a name or the reference body reads as another kind");
}

void VerifyContactSettingsNeedAContactRecord(const ReferenceVehicle& gz18) {
    VehicleProject without_contact = PlacedProjectWithContact(gz18);
    without_contact.contact_record.reset();
    VehicleProjectEditingSession session(without_contact);
    for (const ProjectSettings settings :
         {ProjectSettings::kWheel, ProjectSettings::kRail, ProjectSettings::kContactComputation}) {
        bool refused = false;
        try {
            (void)vehicle_project_editing::ReadSubjectProperties(session.project(), settings);
        } catch (const std::invalid_argument& error) {
            refused = std::string_view(error.what()).find("contact record") != std::string::npos;
        }
        Require(refused, "reading " + Describe(settings) +
                             " without a contact record was not refused for that reason");
        const std::string property_name(
            vehicle_project_editing::DescribeSettingsProperties(settings).front().name);
        RequireRefused(
            session, settings, property_name,
            [&session, settings, &property_name] {
                return session.SetSubjectProperty(settings, property_name, 1.0);
            },
            "contact record", "editing " + Describe(settings) + " without a contact record");
    }
    Require(!vehicle_project_editing::ReadSubjectProperties(session.project(),
                                                            ProjectSettings::kGravity)
                 .empty(),
            "the gravity cannot be read without a contact record");
}

void VerifySettingsSurviveSavingAndReopening(const ReferenceVehicle& gz18,
                                             const std::filesystem::path& directory) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    Require(
        session.SetSubjectProperty(ProjectSettings::kGravity, kGravity, 9.8).has_value() &&
            session
                .SetSubjectProperty(ProjectSettings::kWheel, "nominal_rolling_radius_meters", 0.45)
                .has_value() &&
            session.SetSubjectProperty(ProjectSettings::kRail, "track_gauge_meters", 1.44)
                .has_value() &&
            session
                .SetSubjectProperty(ProjectSettings::kRail, "profile_origin_mode",
                                    PropertyToken{"track_station"})
                .has_value(),
        "a settings edit was refused");
    const VehicleProjectFileLocation file{directory, "settings_probe"};
    session.SaveVehicleProjectAs(file);
    VehicleProjectEditingSession reopened(vehicle_project::ReadVehicleProjectFile(file), file);
    Require(
        NumberOf(reopened.project(), ProjectSettings::kGravity, kGravity) == 9.8 &&
            NumberOf(reopened.project(), ProjectSettings::kWheel,
                     "nominal_rolling_radius_meters") == 0.45 &&
            NumberOf(reopened.project(), ProjectSettings::kRail, "track_gauge_meters") == 1.44 &&
            TokenOf(reopened.project(), ProjectSettings::kRail, "profile_origin_mode") ==
                "track_station",
        "the settings written did not read back after saving and reopening");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 8) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_settings_editing GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT "
                     "WORK_DIRECTORY\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const ReferenceVehicle irw =
            editing_verification::LoadReferenceVehicle(argv[4], argv[5], argv[6]);
        const std::filesystem::path work_directory = argv[7];
        std::filesystem::remove_all(work_directory);
        std::filesystem::create_directories(work_directory);

        VerifyGravity(gz18);
        VerifyWheelRadiusReachesPreview(gz18);
        VerifyRailOriginMode(gz18);
        VerifyReadOnlySettings(gz18);
        VerifyReadOnlySettings(irw);
        VerifyContactSettingsNeedAContactRecord(gz18);
        VerifySettingsSurviveSavingAndReopening(gz18, work_directory);
        std::puts("vehicle project settings editing passed for GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project settings editing: %s\n", error.what());
        return 1;
    }
}
