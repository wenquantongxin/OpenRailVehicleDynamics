#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "vehicle_project/vehicle_project_comparison.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// Saving through the editing session, on GZ18. Whether there are unsaved
// changes follows edits, saves, undo and redo: undoing back to the saved state
// leaves none, and an edit made after undoing past it leaves them until the
// next save. Save needs a file and changes nothing in the project. Save As
// writes the project under the project's name and then gives that name to the
// whole history as the project's identity: it adds no step and a revision only
// when the names change, no undo or redo brings an earlier identity back, and
// the session is associated with the file. A save that cannot write leaves the
// session and the directory as they were.

namespace {

using namespace orvd;
using editing_verification::Fail;
using editing_verification::PlacedProject;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project::VehicleProjectFileLocation;
using vehicle_project_comparison::SameProject;
using vehicle_project_editing::VehicleProjectEditingSession;

const ElementReference kPrimarySuspension{ElementKind::kTranslationalSpringDamper,
                                          "front_leading_left_primary_suspension"};

// One accepted edit: the primary suspension's vertical stiffness.
void EditVerticalStiffness(VehicleProjectEditingSession& session,
                           double stiffness_newtons_per_meter) {
    Require(session
                .SetSubjectProperty(kPrimarySuspension, "stiffness_newtons_per_meter",
                                    Eigen::Vector3d(1.0e6, 1.0e6, stiffness_newtons_per_meter))
                .has_value(),
            "a stiffness edit was refused");
}

void RequireUnsavedChanges(const VehicleProjectEditingSession& session, bool unsaved,
                           const std::string& after) {
    Require(session.has_unsaved_changes() == unsaved,
            std::string("after ") + after + ", the session " +
                (unsaved ? "has no unsaved changes" : "has unsaved changes"));
}

bool SameLocation(const std::optional<VehicleProjectFileLocation>& expected,
                  const std::optional<VehicleProjectFileLocation>& actual) {
    return expected.has_value() == actual.has_value() &&
           (!expected.has_value() ||
            (expected->directory == actual->directory && expected->name == actual->name));
}

// Everything a save may change about a session.
struct SessionState {
    VehicleProject project;
    std::uint64_t revision;
    bool can_undo;
    bool can_redo;
    bool has_unsaved_changes;
    std::optional<VehicleProjectFileLocation> associated_file;
};

SessionState StateOf(const VehicleProjectEditingSession& session) {
    return {session.project(),
            session.revision(),
            session.can_undo(),
            session.can_redo(),
            session.has_unsaved_changes(),
            session.associated_file()};
}

bool SameState(const SessionState& expected, const SessionState& actual) {
    return SameProject(expected.project, actual.project) && expected.revision == actual.revision &&
           expected.can_undo == actual.can_undo && expected.can_redo == actual.can_redo &&
           expected.has_unsaved_changes == actual.has_unsaved_changes &&
           SameLocation(expected.associated_file, actual.associated_file);
}

std::string ReadBytes(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    Require(stream.is_open(), "'" + path.string() + "' cannot be opened");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::vector<std::string> EntriesOf(const std::filesystem::path& directory) {
    std::vector<std::string> entries;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        entries.push_back(entry.path().filename().string());
    }
    std::ranges::sort(entries);
    return entries;
}

// `save` must throw `Refusal` and leave the session as it was.
template <typename Refusal, typename Save>
void RequireSaveRefused(VehicleProjectEditingSession& session, Save save, const std::string& what) {
    const SessionState before = StateOf(session);
    try {
        save();
    } catch (const Refusal&) {
        Require(SameState(before, StateOf(session)), what + " changed the session");
        return;
    }
    Fail(what + " was accepted");
}

void VerifyUnsavedChangesFollowTheHistory(const ReferenceVehicle& gz18,
                                          const std::filesystem::path& directory) {
    VehicleProjectEditingSession session(PlacedProject(gz18));
    RequireUnsavedChanges(session, false, "opening");
    Require(!session.associated_file().has_value(),
            "a session over a vehicle opened from its files has a project file");
    EditVerticalStiffness(session, 2.0e6);
    RequireUnsavedChanges(session, true, "an edit");
    session.Undo();
    RequireUnsavedChanges(session, false, "undoing back to the state opened");
    session.Redo();
    RequireUnsavedChanges(session, true, "redoing the edit");

    const VehicleProjectFileLocation file{directory, "history_probe"};
    session.SaveVehicleProjectAs(file);
    RequireUnsavedChanges(session, false, "Save As");
    // Save, edit, undo: nothing unsaved.
    EditVerticalStiffness(session, 3.0e6);
    RequireUnsavedChanges(session, true, "an edit after saving");
    session.Undo();
    RequireUnsavedChanges(session, false, "undoing the edit made after saving");
    session.Redo();
    RequireUnsavedChanges(session, true, "redoing the edit made after saving");
    session.SaveVehicleProject();
    RequireUnsavedChanges(session, false, "Save");
    Require(SameProject(session.project(), vehicle_project::ReadVehicleProjectFile(file)),
            "Save did not write the project as it is");

    // Save, undo, a new edit: the saved state cannot be reached any more.
    session.Undo();
    RequireUnsavedChanges(session, true, "undoing past the saved state");
    EditVerticalStiffness(session, 4.0e6);
    RequireUnsavedChanges(session, true, "an edit made after undoing past the saved state");
    while (session.can_undo()) {
        session.Undo();
        RequireUnsavedChanges(session, true, "undoing once the saved state is unreachable");
    }
    while (session.can_redo()) {
        session.Redo();
        RequireUnsavedChanges(session, true, "redoing once the saved state is unreachable");
    }
    session.SaveVehicleProject();
    RequireUnsavedChanges(session, false, "the next save");
}

void VerifySaveNeedsAFile(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProject(gz18));
    EditVerticalStiffness(session, 2.0e6);
    RequireSaveRefused<std::logic_error>(
        session, [&session] { session.SaveVehicleProject(); }, "Save without a project file");
}

// The project as it would be under the project name `name`.
VehicleProject UnderProjectName(VehicleProject project, const std::string& name) {
    project.vehicle_definition.vehicle_name = name;
    project.vehicle_definition.mechanical_definition_identifier = name;
    return project;
}

// Save As writes the project under the project's name, then gives that name to
// the current project and to every project of the history, which keeps its
// steps: no undo or redo brings an earlier identity back, so a Save made after
// undoing writes the names again.
void VerifySaveAsGivesTheWholeHistoryItsNames(const ReferenceVehicle& gz18,
                                              const std::filesystem::path& directory) {
    const VehicleProject opened = PlacedProject(gz18);
    VehicleProjectEditingSession session(opened);
    EditVerticalStiffness(session, 2.0e6);
    EditVerticalStiffness(session, 3.0e6);
    session.Undo();
    const VehicleProject before = session.project();
    const std::uint64_t revision = session.revision();
    const VehicleProjectFileLocation file{directory, "renamed_gz18"};
    session.SaveVehicleProjectAs(file);

    Require(SameProject(UnderProjectName(before, file.name), session.project()),
            "Save As did not give the vehicle the project's name, or changed more");
    Require(SameProject(UnderProjectName(before, file.name),
                        vehicle_project::ReadVehicleProjectFile(file)),
            "Save As did not write the project under the project's name");
    Require(session.revision() == revision + 1 && session.can_undo() && session.can_redo(),
            "a Save As that renames did not add exactly one revision and keep the history's steps");
    Require(SameLocation(file, session.associated_file()) && !session.has_unsaved_changes(),
            "Save As did not associate the session with the file and mark its state saved");

    const auto require_project_names = [&file](const VehicleProjectEditingSession& renamed,
                                               const std::string& after) {
        const auto& vehicle = renamed.project().vehicle_definition;
        Require(vehicle.vehicle_name == file.name &&
                    vehicle.mechanical_definition_identifier == file.name,
                after + " brought back an identity the project had before Save As");
    };
    session.Redo();
    require_project_names(session, "redoing after Save As");
    while (session.can_undo()) {
        session.Undo();
        require_project_names(session, "undoing after Save As");
    }
    Require(SameProject(UnderProjectName(opened, file.name), session.project()) &&
                session.has_unsaved_changes(),
            "undoing every step after Save As did not give the project opened, under the "
            "project's names and with unsaved changes");

    // Edit, Save As, undo, Save: the file keeps the project's names.
    session.SaveVehicleProject();
    Require(SameProject(UnderProjectName(opened, file.name),
                        vehicle_project::ReadVehicleProjectFile(file)),
            "Save after undoing past Save As did not write the project under the project's "
            "names");
}

// Save As adds no step whatever the names were, and a revision only when
// they change.
void VerifySaveAsUnderTheVehicleNames(const ReferenceVehicle& gz18,
                                      const std::filesystem::path& directory) {
    const VehicleProject named = UnderProjectName(PlacedProject(gz18), "named_gz18");
    VehicleProjectEditingSession session(named);
    session.SaveVehicleProjectAs({directory, "named_gz18"});
    Require(
        session.revision() == 0 && !session.can_undo() && !session.has_unsaved_changes() &&
            SameProject(named, session.project()) &&
            SameProject(named, vehicle_project::ReadVehicleProjectFile({directory, "named_gz18"})),
        "Save As under the names the vehicle already has did more than write and associate");

    VehicleProject half_named = PlacedProject(gz18);
    half_named.vehicle_definition.vehicle_name = "half_named_gz18";
    VehicleProjectEditingSession half_named_session(half_named);
    half_named_session.SaveVehicleProjectAs({directory, "half_named_gz18"});
    Require(half_named_session.revision() == 1 && !half_named_session.can_undo() &&
                SameProject(UnderProjectName(half_named, "half_named_gz18"),
                            half_named_session.project()),
            "Save As under the vehicle's name alone did not give it the identifier as one "
            "revision without a step");
}

// Saves that cannot write: a name that is not one, a directory that does not
// exist, a project the file cannot hold, a file whose directory has gone.
void VerifyFailedSavesChangeNothing(const ReferenceVehicle& gz18,
                                    const std::filesystem::path& directory) {
    VehicleProjectEditingSession session(PlacedProject(gz18));
    const VehicleProjectFileLocation saved{directory, "saved_before_failures"};
    session.SaveVehicleProjectAs(saved);
    EditVerticalStiffness(session, 2.0e6);
    const std::string saved_bytes = ReadBytes(vehicle_project::VehicleProjectFilePath(saved));
    const std::vector<std::string> entries = EntriesOf(directory);

    RequireSaveRefused<std::invalid_argument>(
        session, [&] { session.SaveVehicleProjectAs({directory, "renamed gz18"}); },
        "Save As under a name with a space");
    const std::filesystem::path missing_directory = directory / "missing_directory";
    RequireSaveRefused<std::runtime_error>(
        session, [&] { session.SaveVehicleProjectAs({missing_directory, "renamed_gz18"}); },
        "Save As into a directory that does not exist");
    Require(ReadBytes(vehicle_project::VehicleProjectFilePath(saved)) == saved_bytes &&
                EntriesOf(directory) == entries && !std::filesystem::exists(missing_directory),
            "a refused Save As changed the directory");

    VehicleProject without_gravity = PlacedProject(gz18);
    without_gravity.gravitational_acceleration_meters_per_second_squared.reset();
    VehicleProjectEditingSession without_gravity_session(without_gravity);
    RequireSaveRefused<std::invalid_argument>(
        without_gravity_session,
        [&] { without_gravity_session.SaveVehicleProjectAs({directory, "without_gravity"}); },
        "Save As of a project stating no gravity");
    Require(EntriesOf(directory) == entries, "a refused Save As changed the directory");

    const std::filesystem::path removed_directory = directory / "removed_directory";
    std::filesystem::create_directory(removed_directory);
    session.SaveVehicleProjectAs({removed_directory, "saved_then_removed"});
    EditVerticalStiffness(session, 3.0e6);
    std::filesystem::remove_all(removed_directory);
    RequireSaveRefused<std::runtime_error>(
        session, [&] { session.SaveVehicleProject(); },
        "Save to a file whose directory has been removed");
    Require(!std::filesystem::exists(removed_directory),
            "a refused Save created the removed directory again");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_editing_session_saving GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT WORK_DIRECTORY\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const std::filesystem::path work_directory = argv[4];
        std::filesystem::remove_all(work_directory);
        std::filesystem::create_directories(work_directory);

        VerifyUnsavedChangesFollowTheHistory(gz18, work_directory);
        VerifySaveNeedsAFile(gz18);
        VerifySaveAsGivesTheWholeHistoryItsNames(gz18, work_directory);
        VerifySaveAsUnderTheVehicleNames(gz18, work_directory);
        VerifyFailedSavesChangeNothing(gz18, work_directory);
        std::puts(
            "vehicle project editing session saving: unsaved changes, Save and Save As passed "
            "for GZ18");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project editing session saving: %s\n", error.what());
        return 1;
    }
}
