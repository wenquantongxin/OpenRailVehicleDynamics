#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <expected>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <limits>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "vehicle_project/vehicle_project_comparison.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// The three orientations a project holds, read and edited as roll, pitch and
// yaw in radians. On both bundled vehicles as the modelling service opens
// them, every rotation property reads the matrix its record holds, bit for
// bit, and angles that give it, and reading changes nothing. A marker's
// orientation, a carrier's and a free body's placement rotation are set, read
// back, and found in the preview where they are drawn; each edit is one undo
// step and an unsaved change; angles that are not finite are refused, naming
// the property; a body hanging on a joint has no placement rotation and is
// refused one. A marker is created with an orientation or aligned with its
// body. A project with edited orientations is saved and reopened equal, and
// one whose orientations were only read saves to the same bytes. A record
// holding no rotation, a marker's matrix that is not orthonormal or a
// placement's quaternion not of unit length, is read as it is, its angles
// absent, is reported by the check, and is repaired by three angles, after
// which the check passes; whether a record holds a rotation agrees with the
// check on either side of its tolerance.

namespace {

using namespace orvd;
using editing_verification::PlacedProjectWithContact;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using editing_verification::RequireAccepted;
using editing_verification::RequireCheckPasses;
using editing_verification::RequireRefused;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project::VehicleProjectFileLocation;
using vehicle_project_comparison::SameProject;
using vehicle_project_editing::PropertyOrientation;
using vehicle_project_editing::PropertyReading;
using vehicle_project_editing::PropertyValueKind;
using vehicle_project_editing::VehicleProjectEditingSession;
using vehicle_project_editing::VehicleProjectPreview;

constexpr double kPi = std::numbers::pi;

const ElementReference kMarker{ElementKind::kFixedFrame, "front_coupler_mount"};
const std::string kMarkerRotation = "rotation_in_body_frame";
const std::string kPlacementRotation = "placement.rotation_local_track_from_body";

// Rz(yaw) Ry(pitch) Rx(roll), each factor written out here.
Eigen::Matrix3d RotationWrittenOut(const Eigen::Vector3d& roll_pitch_yaw_radians) {
    const double roll = roll_pitch_yaw_radians.x();
    const double pitch = roll_pitch_yaw_radians.y();
    const double yaw = roll_pitch_yaw_radians.z();
    Eigen::Matrix3d about_z;
    about_z << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d about_y;
    about_y << std::cos(pitch), 0.0, std::sin(pitch), 0.0, 1.0, 0.0, -std::sin(pitch), 0.0,
        std::cos(pitch);
    Eigen::Matrix3d about_x;
    about_x << 1.0, 0.0, 0.0, 0.0, std::cos(roll), -std::sin(roll), 0.0, std::sin(roll),
        std::cos(roll);
    return about_z * about_y * about_x;
}

std::string Scientific(double value) {
    std::ostringstream text;
    text << std::scientific << value;
    return text.str();
}

// A few rounding steps of a product of rotations, of a drawing's pose and of
// angles read from a matrix.
template <typename Expected, typename Actual>
bool Near(const Eigen::MatrixBase<Expected>& expected, const Eigen::MatrixBase<Actual>& actual) {
    return (expected - actual).cwiseAbs().maxCoeff() <= 1.0e-12;
}

PropertyOrientation Angles(const Eigen::Vector3d& roll_pitch_yaw_radians) {
    return {.roll_pitch_yaw_radians = roll_pitch_yaw_radians};
}

PropertyReading ReadingOf(const VehicleProject& project, const ElementReference& element,
                          const std::string& property_name) {
    for (const PropertyReading& reading :
         vehicle_project_editing::ReadSubjectProperties(project, element)) {
        if (reading.description.name == property_name) {
            return reading;
        }
    }
    editing_verification::Fail("'" + element.name + "' has no property '" + property_name + "'");
}

// The orientation a rotation property reads, which must have a value.
PropertyOrientation OrientationOf(const VehicleProject& project, const ElementReference& element,
                                  const std::string& property_name) {
    const PropertyReading reading = ReadingOf(project, element, property_name);
    Require(
        reading.value.has_value() && std::holds_alternative<PropertyOrientation>(*reading.value),
        "'" + element.name + "' " + property_name + " reads no orientation");
    return std::get<PropertyOrientation>(*reading.value);
}

// The orientation reads `angles` back and a matrix those angles give.
void RequireReadsBack(const VehicleProject& project, const ElementReference& element,
                      const std::string& property_name, const Eigen::Vector3d& angles) {
    const PropertyOrientation read = OrientationOf(project, element, property_name);
    Require(read.matrix.has_value() && read.roll_pitch_yaw_radians.has_value() &&
                Near(angles, *read.roll_pitch_yaw_radians) &&
                Near(RotationWrittenOut(angles), *read.matrix),
            "'" + element.name + "' " + property_name +
                " does not read back the angles set and the rotation they give");
}

const vehicle_project::FreeBodyPlacement& PlacementOf(const VehicleProject& project,
                                                      const std::string& body_name) {
    for (const vehicle_project::FreeBodyPlacement& placement : project.free_body_placements) {
        if (placement.body_name == body_name) {
            return placement;
        }
    }
    editing_verification::Fail("'" + body_name + "' has no placement");
}

const configuration::VehicleFixedFrameDefinition& FrameOf(const VehicleProject& project,
                                                          const std::string& frame_name) {
    return *vehicle_project::FindElementRecord<
        vehicle_project::ElementKindRecords<ElementKind::kFixedFrame>>(project, frame_name);
}

// The matrix the record behind a rotation property holds, as the record
// states it: a matrix, or the matrix of a placement's quaternion.
std::optional<Eigen::Matrix3d> RecordedMatrix(const VehicleProject& project,
                                              const ElementReference& element,
                                              const std::string& property_name) {
    if (element.kind == ElementKind::kFixedFrame) {
        return FrameOf(project, element.name).rotation_in_body_frame;
    }
    if (element.kind == ElementKind::kContactCarrier) {
        return vehicle_project::FindElementRecord<
                   vehicle_project::ElementKindRecords<ElementKind::kContactCarrier>>(project,
                                                                                      element.name)
            ->rotation_body_from_nonspinning_wheel_profile;
    }
    Require(property_name == kPlacementRotation, "'" + property_name + "' is no rotation here");
    return PlacementOf(project, element.name).rotation_local_track_from_body.toRotationMatrix();
}

// --- The bundled vehicles ------------------------------------------------------

// Every rotation property reads the matrix its record holds, bit for bit, and
// angles that give it; reading every property changes nothing.
void VerifyBundledVehicleOrientationsRead(const ReferenceVehicle& reference) {
    const VehicleProject opened = PlacedProjectWithContact(reference);
    VehicleProjectEditingSession session(opened);
    const std::string& vehicle_name = reference.definition.vehicle_name;
    std::size_t turned = 0;
    for (const ElementKind kind : vehicle_project::kElementKinds) {
        for (const std::string& name :
             vehicle_project_editing::CollectElementNames(session.project(), kind)) {
            const ElementReference element{kind, name};
            for (const PropertyReading& reading :
                 vehicle_project_editing::ReadSubjectProperties(session.project(), element)) {
                if (reading.description.value_kind != PropertyValueKind::kRotation ||
                    !reading.value.has_value()) {
                    continue;
                }
                const std::string property_name(reading.description.name);
                const PropertyOrientation& read = std::get<PropertyOrientation>(*reading.value);
                const std::string what = vehicle_name + " '" + name + "' " + property_name;
                Require(read.matrix.has_value() && read.roll_pitch_yaw_radians.has_value() &&
                            read.matrix == RecordedMatrix(opened, element, property_name),
                        what + " does not read its record's matrix, bit for bit, with angles");
                Require(Near(RotationWrittenOut(*read.roll_pitch_yaw_radians), *read.matrix),
                        what + " reads angles that do not give its matrix");
                turned += read.roll_pitch_yaw_radians->isZero() ? 0 : 1;
            }
        }
    }
    Require(SameProject(opened, session.project()),
            vehicle_name + ": reading every property changed the project");
    if (vehicle_name == "IRW") {
        Require(turned > 0, "IRW holds no orientation that is turned");
    }
}

// --- Editing ---------------------------------------------------------------

// The edit was accepted as one undo step and an unsaved change.
template <typename Value>
void RequireOneUnsavedEdit(
    const VehicleProjectEditingSession& session,
    const std::expected<Value, vehicle_project_editing::SubjectEditRefusal>& outcome,
    std::uint64_t revision_before, const std::string& what) {
    RequireAccepted(outcome, what);
    Require(session.revision() == revision_before + 1 && session.can_undo() &&
                session.has_unsaved_changes(),
            what + " was not one undo step and an unsaved change");
}

// A marker on GZ18's car body turned: it reads the angles back, the preview
// draws its frame turned so in the body's, and one undo restores the project.
void VerifyMarkerOrientationEdited(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const Eigen::Vector3d angles(0.2, -0.4, 1.1);
    RequireOneUnsavedEdit(session,
                          session.SetSubjectProperty(kMarker, kMarkerRotation, Angles(angles)), 0,
                          "turning a marker");
    RequireReadsBack(session.project(), kMarker, kMarkerRotation, angles);

    RequireCheckPasses(session, "GZ18 with a turned marker");
    const VehicleProjectPreview& preview = *session.preview();
    const std::string& body_name = FrameOf(session.project(), kMarker.name).body_name;
    const auto body = std::ranges::find(preview.rigid_bodies, body_name,
                                        &vehicle_project_editing::RigidBodyDrawing::body_name);
    const auto frame = std::ranges::find(preview.fixed_frames, kMarker.name,
                                         &vehicle_project_editing::FixedFrameDrawing::frame_name);
    Require(body != preview.rigid_bodies.end() && frame != preview.fixed_frames.end() &&
                Near(body->rotation_drawing_from_body * RotationWrittenOut(angles),
                     frame->rotation_drawing_from_frame),
            "the preview does not draw the turned marker turned so in its body");

    session.Undo();
    Require(SameProject(opened, session.project()), "undoing the turn did not restore GZ18");
}

// Angles that are not finite are refused, naming the property, and change
// nothing.
void VerifyAnglesThatAreNotFiniteRefused(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject before = session.project();
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d& angles :
         {Eigen::Vector3d(not_a_number, 0.0, 0.0), Eigen::Vector3d(0.0, infinity, 0.0),
          Eigen::Vector3d(0.0, 0.0, -infinity)}) {
        RequireRefused(session.SetSubjectProperty(kMarker, kMarkerRotation, Angles(angles)),
                       kMarker, kMarkerRotation, session, before, 0, false,
                       "turning a marker by an angle that is not finite");
    }
}

// A contact carrier's wheel-profile basis turned reads the angles back.
void VerifyCarrierOrientationEdited(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const ElementReference carrier{ElementKind::kContactCarrier,
                                   gz18.contact_record.declaration.carriers.front().carrier_name};
    const std::string property_name = "rotation_body_from_nonspinning_wheel_profile";
    const Eigen::Vector3d angles(kPi, 0.0, 0.0);
    RequireOneUnsavedEdit(session,
                          session.SetSubjectProperty(carrier, property_name, Angles(angles)), 0,
                          "turning a carrier's wheel-profile basis");
    RequireReadsBack(session.project(), carrier, property_name, angles);
}

// A free bogie frame's placement turned: its quaternion is of unit length, it
// reads the angles back, and the preview draws the body turned so. A body
// hanging on a joint has no placement rotation and is refused one.
void VerifyPlacementRotationEdited(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const VehicleProject opened = session.project();
    const ElementReference bogie{ElementKind::kRigidBody, "front_bogie_frame"};
    const Eigen::Vector3d angles(0.05, -0.02, 0.3);
    RequireOneUnsavedEdit(session,
                          session.SetSubjectProperty(bogie, kPlacementRotation, Angles(angles)), 0,
                          "turning a free body's placement");
    RequireReadsBack(session.project(), bogie, kPlacementRotation, angles);
    Require(
        std::abs(PlacementOf(session.project(), bogie.name).rotation_local_track_from_body.norm() -
                 1.0) <= 1.0e-15,
        "a turned placement does not hold a unit quaternion");
    RequireCheckPasses(session, "GZ18 with a turned bogie frame");
    const VehicleProjectPreview& preview = *session.preview();
    const auto drawn = std::ranges::find(preview.rigid_bodies, bogie.name,
                                         &vehicle_project_editing::RigidBodyDrawing::body_name);
    Require(drawn != preview.rigid_bodies.end() &&
                Near(RotationWrittenOut(angles), drawn->rotation_drawing_from_body),
            "the preview does not draw the bogie frame turned as its placement is");
    session.Undo();
    Require(SameProject(opened, session.project()), "undoing the turn did not restore GZ18");

    VehicleProjectEditingSession untouched(opened);
    const ElementReference carrier_body{ElementKind::kRigidBody,
                                        "front_leading_right_axlebox_carrier"};
    const PropertyReading none = ReadingOf(untouched.project(), carrier_body, kPlacementRotation);
    Require(!none.value.has_value() && !none.description.editable,
            "a body hanging on a joint has a placement rotation, or reads one as editable");
    RequireRefused(
        untouched.SetSubjectProperty(carrier_body, kPlacementRotation, Angles({0.1, 0.0, 0.0})),
        carrier_body, kPlacementRotation, untouched, opened, 0, false,
        "turning the placement of a body hanging on a joint");
}

// A marker is created turned, as its angles give, or aligned with its body.
void VerifyMarkerCreatedWithAndWithoutOrientation(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProjectWithContact(gz18));
    const Eigen::Vector3d quarter_turn_of_yaw(0.0, 0.0, kPi / 2.0);
    RequireAccepted(session.AddElement(
                        ElementKind::kFixedFrame, "turned_marker",
                        {{"body_name", vehicle_project_editing::ReferencedElementName{"carbody"}},
                         {kMarkerRotation, Angles(quarter_turn_of_yaw)}}),
                    "creating a turned marker");
    RequireAccepted(session.AddElement(
                        ElementKind::kFixedFrame, "aligned_marker",
                        {{"body_name", vehicle_project_editing::ReferencedElementName{"carbody"}}}),
                    "creating a marker without its orientation");
    const Eigen::Matrix3d turned =
        FrameOf(session.project(), "turned_marker").rotation_in_body_frame;
    Require(Near(Eigen::Vector3d::UnitY(), turned * Eigen::Vector3d::UnitX()) &&
                Near(-Eigen::Vector3d::UnitX(), turned * Eigen::Vector3d::UnitY()),
            "a marker created a quarter turn of yaw from its body does not take x to y");
    Require(FrameOf(session.project(), "aligned_marker").rotation_in_body_frame ==
                Eigen::Matrix3d::Identity(),
            "a marker created without its orientation is not aligned with its body");
}

// --- Saving ----------------------------------------------------------------

std::string ReadBytes(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    Require(stream.is_open(), "'" + path.string() + "' cannot be opened");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

// Reads every property of every element, as an interface shows a project.
void ReadEveryProperty(const VehicleProject& project) {
    for (const ElementKind kind : vehicle_project::kElementKinds) {
        for (const std::string& name :
             vehicle_project_editing::CollectElementNames(project, kind)) {
            (void)vehicle_project_editing::ReadSubjectProperties(project,
                                                                 ElementReference{kind, name});
        }
    }
}

// A project with edited orientations is saved and reopened equal; one whose
// orientations were only read saves to the same bytes as before.
void VerifyOrientationsSavedAndReopened(const ReferenceVehicle& gz18,
                                        const std::filesystem::path& directory) {
    VehicleProjectEditingSession edited(PlacedProjectWithContact(gz18));
    RequireAccepted(edited.SetSubjectProperty(kMarker, kMarkerRotation, Angles({0.1, 0.2, 0.3})),
                    "turning a marker");
    RequireAccepted(
        edited.SetSubjectProperty(ElementReference{ElementKind::kRigidBody, "rear_bogie_frame"},
                                  kPlacementRotation, Angles({0.0, 0.0, -0.4})),
        "turning a placement");
    const VehicleProjectFileLocation edited_file{directory, "edited_orientations"};
    edited.SaveVehicleProjectAs(edited_file);
    Require(SameProject(edited.project(), vehicle_project::ReadVehicleProjectFile(edited_file)),
            "a project with edited orientations does not reopen equal");

    const VehicleProjectFileLocation read_file{directory, "read_orientations"};
    vehicle_project::WriteVehicleProjectFile(PlacedProjectWithContact(gz18), read_file);
    const std::string written = ReadBytes(vehicle_project::VehicleProjectFilePath(read_file));
    VehicleProjectEditingSession read(vehicle_project::ReadVehicleProjectFile(read_file),
                                      read_file);
    ReadEveryProperty(read.project());
    read.SaveVehicleProject();
    Require(ReadBytes(vehicle_project::VehicleProjectFilePath(read_file)) == written,
            "a project whose orientations were only read does not save to the same bytes");
}

// --- Records holding no rotation ---------------------------------------------

// GZ18 with `matrix` in a marker and `quaternion` as the car body's placement
// rotation, as a project file may hold them.
VehicleProject Gz18Holding(const ReferenceVehicle& gz18, const Eigen::Matrix3d& matrix,
                           const Eigen::Quaterniond& quaternion) {
    VehicleProject project = PlacedProjectWithContact(gz18);
    for (configuration::VehicleFixedFrameDefinition& frame :
         project.vehicle_definition.fixed_frames) {
        if (frame.name == kMarker.name) {
            frame.rotation_in_body_frame = matrix;
        }
    }
    for (vehicle_project::FreeBodyPlacement& placement : project.free_body_placements) {
        if (placement.body_name == "carbody") {
            placement.rotation_local_track_from_body = quaternion;
        }
    }
    return project;
}

// A marker's matrix that is not orthonormal and a placement's quaternion not of
// unit length are read as they are, their angles absent and the quaternion's
// matrix too, and stay editable; reading changes nothing. The check reports
// each, three angles repair each, and the check then passes.
void VerifyRecordsHoldingNoRotationRepaired(const ReferenceVehicle& gz18) {
    const Eigen::Matrix3d stretched = Eigen::Vector3d(2.0, 1.0, 1.0).asDiagonal();
    const VehicleProject opened =
        Gz18Holding(gz18, stretched, Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0));
    VehicleProjectEditingSession session(opened);
    const ElementReference car_body{ElementKind::kRigidBody, "carbody"};

    const PropertyReading marker = ReadingOf(session.project(), kMarker, kMarkerRotation);
    Require(marker.value.has_value() && marker.description.editable &&
                std::get<PropertyOrientation>(*marker.value) ==
                    PropertyOrientation{.matrix = stretched},
            "a marker's matrix that is not orthonormal is not read as it is, without angles and "
            "editable");
    const PropertyReading placement = ReadingOf(session.project(), car_body, kPlacementRotation);
    Require(placement.value.has_value() && placement.description.editable &&
                std::get<PropertyOrientation>(*placement.value) == PropertyOrientation{},
            "a placement's quaternion not of unit length is not read without matrix and angles, "
            "editable");
    ReadEveryProperty(session.project());
    Require(SameProject(opened, session.project()),
            "reading records holding no rotation changed them");

    Require(!session.CheckMechanics().refusals.empty(),
            "the check passes a marker's matrix that is not orthonormal");
    RequireAccepted(session.SetSubjectProperty(kMarker, kMarkerRotation, Angles({0.0, 0.0, 0.0})),
                    "repairing the marker's matrix by three angles");
    Require(!session.CheckMechanics().refusals.empty(),
            "the check passes a placement's quaternion not of unit length");
    RequireAccepted(
        session.SetSubjectProperty(car_body, kPlacementRotation, Angles({0.0, 0.0, 0.0})),
        "repairing the placement's quaternion by three angles");
    RequireCheckPasses(session, "GZ18 with both records repaired by three angles");
}

// Whether a record holds a rotation is judged as the check judges it, on
// either side of the check's tolerance: within it, the record reads angles
// and the check passes; beyond it, no angles and the check refuses. The check
// holds a marker's matrix to the multibody layer's tolerance, 128 machine
// epsilons on R R^T, and a placement's quaternion to 1e-12 on its norm.
void VerifyRotationJudgedAsTheCheckJudgesIt(const ReferenceVehicle& gz18) {
    const auto reads_angles = [](const VehicleProject& project, const ElementReference& element,
                                 const std::string& property_name) {
        return OrientationOf(project, element, property_name).roll_pitch_yaw_radians.has_value();
    };
    const ElementReference car_body{ElementKind::kRigidBody, "carbody"};
    const Eigen::Quaterniond unit = Eigen::Quaterniond::Identity();
    for (const double stretch : {1.0e-14, 1.0e-13}) {
        const Eigen::Matrix3d matrix = Eigen::Vector3d(1.0 + stretch, 1.0, 1.0).asDiagonal();
        VehicleProjectEditingSession session(Gz18Holding(gz18, matrix, unit));
        Require(reads_angles(session.project(), kMarker, kMarkerRotation) ==
                    session.CheckMechanics().refusals.empty(),
                "a marker's matrix stretched by " + Scientific(stretch) +
                    " reads angles exactly when the check passes it, or the reverse");
    }
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    for (const double excess : {0.5e-12, 2.0e-12}) {
        VehicleProjectEditingSession session(
            Gz18Holding(gz18, identity, Eigen::Quaterniond(1.0 + excess, 0.0, 0.0, 0.0)));
        Require(reads_angles(session.project(), car_body, kPlacementRotation) ==
                    session.CheckMechanics().refusals.empty(),
                "a placement's quaternion whose norm exceeds one by " + Scientific(excess) +
                    " reads angles exactly when the check passes it, or the reverse");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 8) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_orientation_editing GZ18_VEHICLE "
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
        VerifyBundledVehicleOrientationsRead(gz18);
        VerifyBundledVehicleOrientationsRead(irw);
        VerifyMarkerOrientationEdited(gz18);
        VerifyAnglesThatAreNotFiniteRefused(gz18);
        VerifyCarrierOrientationEdited(gz18);
        VerifyPlacementRotationEdited(gz18);
        VerifyMarkerCreatedWithAndWithoutOrientation(gz18);
        VerifyOrientationsSavedAndReopened(gz18, work_directory);
        VerifyRecordsHoldingNoRotationRepaired(gz18);
        VerifyRotationJudgedAsTheCheckJudgesIt(gz18);
        std::puts("vehicle project orientation editing passed for GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project orientation editing: %s\n", error.what());
        return 1;
    }
}
