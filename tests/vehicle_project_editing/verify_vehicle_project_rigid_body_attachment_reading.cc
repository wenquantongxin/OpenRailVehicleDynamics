#include <algorithm>
#include <cstdio>
#include <exception>
#include <iterator>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"
#include "orvd/vehicle_project_editing/vehicle_project_titles_in_english_and_chinese.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// How rigid bodies are attached, read from the records alone. On both bundled
// vehicles as the modelling service opens them: every body stating that it
// moves freely reads free, and only the station layout's reference body cannot
// take a joint; every other body hangs on the joint the vehicle's file gives
// it, the one whose child end is on it, held at that joint's other end. A new
// joint's end on a body may name the body and its fixed frames, and its other
// end every frame on a body outside the body's branch. On records a project
// file can hold: a joint whose parent end is on the hanging body reads the
// same body as hanging; a lone body that is not free is unattached; a group
// without a free body is unattached, body by body; a loop, and a group with
// two free bodies, are conflicting, every body of them, and change nothing;
// and a joint with an end naming no frame, or with both ends on one body,
// relates nothing.

namespace {

using namespace orvd;
using editing_verification::PlacedProjectWithContact;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project_editing::ReadRigidBodyAttachment;
using vehicle_project_editing::RigidBodyAttachment;
using vehicle_project_editing::RigidBodyAttachmentState;
using vehicle_project_editing::Title;

// The texts the contract gives a reading.
struct ContractTexts {
    Title free{.en = "Moves freely in the world", .zh = "相对世界自由"};
    Title hanging{.en = "Hangs on a joint", .zh = "由铰挂接"};
    Title reference_body_note{
        .en = "The station layout is measured from this body, so it keeps moving freely",
        .zh = "站位布局以它为基准，它须保持自由"};
    Title not_attached{.en = "Not attached", .zh = "未接入"};
    Title not_attached_note{.en = "Neither free nor held by a joint",
                            .zh = "既没有声明自由，也没有铰把它接到别的刚体上"};
    Title unclear{.en = "Attachment unclear", .zh = "挂接不明"};
    Title loop_note{.en = "The joints around it close a loop; remove one of them",
                    .zh = "它周围的铰连成了环，请删除其中一个"};
    Title several_free_bodies_note{
        .en =
            "It is joined through joints to more than one free body; remove one of the joints "
            "between them",
        .zh = "它经由铰连到了不止一个自由刚体，请删除其间的一个铰"};
};

// A joint record's name and ends, whatever its kind.
struct JointEnds {
    ElementReference joint;
    std::string parent_frame_name;
    std::string child_frame_name;
};

std::vector<JointEnds> JointsOf(const VehicleProject& project) {
    const configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    std::vector<JointEnds> joints;
    for (const auto& joint : vehicle.revolute_joints) {
        joints.push_back({{ElementKind::kRevoluteJoint, joint.name},
                          joint.parent_frame_name,
                          joint.child_frame_name});
    }
    for (const auto& joint : vehicle.ball_rpy_joints) {
        joints.push_back({{ElementKind::kBallRpyJoint, joint.name},
                          joint.parent_frame_name,
                          joint.child_frame_name});
    }
    for (const auto& joint : vehicle.weld_joints) {
        joints.push_back({{ElementKind::kWeldJoint, joint.name},
                          joint.parent_frame_name,
                          joint.child_frame_name});
    }
    return joints;
}

// The body a frame of a bundled vehicle is on: the body of that name, or the
// body the fixed frame of that name names.
std::string BodyOfFrame(const VehicleProject& project, const std::string& frame_name) {
    const auto& frames = project.vehicle_definition.fixed_frames;
    const auto frame =
        std::ranges::find(frames, frame_name, &configuration::VehicleFixedFrameDefinition::name);
    return frame == frames.end() ? frame_name : frame->body_name;
}

// The frame of that name in a bundled vehicle, a rigid body or a fixed frame.
ElementReference FrameNamed(const VehicleProject& project, const std::string& frame_name) {
    const auto& bodies = project.vehicle_definition.rigid_bodies;
    const bool body =
        std::ranges::find(bodies, frame_name, &configuration::VehicleRigidBodyDefinition::name) !=
        bodies.end();
    return {body ? ElementKind::kRigidBody : ElementKind::kFixedFrame, frame_name};
}

// `body`'s own frame, then the fixed frames naming it in declaration order.
std::vector<ElementReference> FramesOn(const VehicleProject& project, const std::string& body) {
    std::vector<ElementReference> frames{{ElementKind::kRigidBody, body}};
    for (const configuration::VehicleFixedFrameDefinition& frame :
         project.vehicle_definition.fixed_frames) {
        if (frame.body_name == body) {
            frames.push_back({ElementKind::kFixedFrame, frame.name});
        }
    }
    return frames;
}

// Every frame on a body not in `left_out`, body by body in declaration order.
std::vector<ElementReference> FramesOutside(const VehicleProject& project,
                                            const std::set<std::string>& left_out) {
    std::vector<ElementReference> frames;
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        if (!left_out.contains(body.name)) {
            std::ranges::copy(FramesOn(project, body.name), std::back_inserter(frames));
        }
    }
    return frames;
}

// `body` and the bodies hanging below it in a bundled vehicle, whose joints
// all run from their parent end towards the body that hangs on them.
std::set<std::string> Branch(const VehicleProject& project, const std::string& body) {
    std::set<std::string> branch{body};
    for (bool grew = true; grew;) {
        grew = false;
        for (const JointEnds& joint : JointsOf(project)) {
            if (branch.contains(BodyOfFrame(project, joint.parent_frame_name)) &&
                branch.insert(BodyOfFrame(project, joint.child_frame_name)).second) {
                grew = true;
            }
        }
    }
    return branch;
}

void VerifyBundledVehicle(const VehicleProject& project) {
    const ContractTexts texts;
    const configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    const std::vector<JointEnds> joints = JointsOf(project);
    for (const configuration::VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        const RigidBodyAttachment attachment = ReadRigidBodyAttachment(project, body.name);
        const std::string what = vehicle.vehicle_name + ": rigid body '" + body.name + "'";
        Require(attachment.frames_on_body == FramesOn(project, body.name),
                what + ": what a new joint's end on it may name is not it and its fixed frames");
        Require(attachment.frames_it_can_be_held_at ==
                    FramesOutside(project, Branch(project, body.name)),
                what +
                    ": what a new joint's other end may name is not every frame outside its "
                    "branch");
        if (body.moves_freely_in_world) {
            const bool reference =
                body.name == vehicle.mechanical_track_station_layout.reference_body_name;
            Require(attachment.state == RigidBodyAttachmentState::kFree &&
                        attachment.summary == texts.free && !attachment.joint.has_value() &&
                        !attachment.held_at.has_value() && !attachment.can_become_free &&
                        attachment.can_take_joint == !reference &&
                        attachment.note ==
                            (reference ? std::optional(texts.reference_body_note) : std::nullopt),
                    what +
                        " states that it moves freely but does not read free, able to take a "
                        "joint unless it is the station layout's reference body, which says why");
            continue;
        }
        const auto given = std::ranges::find_if(joints, [&](const JointEnds& joint) {
            return BodyOfFrame(project, joint.child_frame_name) == body.name;
        });
        Require(given != joints.end(), what + " has no joint whose child end is on it");
        Require(attachment.state == RigidBodyAttachmentState::kHangsOnJoint &&
                    attachment.summary == texts.hanging && attachment.joint == given->joint &&
                    attachment.held_at == FrameNamed(project, given->parent_frame_name) &&
                    !attachment.note.has_value() && attachment.can_become_free &&
                    attachment.can_take_joint,
                what + " does not read as hanging on " +
                    vehicle_project::DescribeElement(given->joint) + ", held at '" +
                    given->parent_frame_name + "', able to become free and take a joint");
    }
}

// --- Records a project file can hold -----------------------------------------

configuration::VehicleRigidBodyDefinition Body(const std::string& name, bool moves_freely) {
    return {.name = name, .moves_freely_in_world = moves_freely, .mass_kilograms = 1.0};
}

// One project of several groups, each attached in its own way:
// - chain_root is free; chain_middle hangs on chain_pivot below it, whose
//   child end is on chain_middle; chain_end hangs on reversed_weld below
//   chain_middle, whose parent end is on chain_end;
// - lone is not free and related to nothing: dangling_ball names a frame the
//   project does not have, and self_weld has both ends on lone;
// - pair_first and pair_second, neither free, are welded together;
// - loop_root is free, and loop_first and loop_second close a loop with it;
// - first_free and second_free are free, and between hangs between them.
VehicleProject ProjectOfSeveralGroups() {
    VehicleProject project;
    configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    vehicle.vehicle_name = "several groups";
    vehicle.rigid_bodies = {
        Body("chain_root", true), Body("chain_middle", false), Body("chain_end", false),
        Body("lone", false),      Body("pair_first", false),   Body("pair_second", false),
        Body("loop_root", true),  Body("loop_first", false),   Body("loop_second", false),
        Body("first_free", true), Body("between", false),      Body("second_free", true)};
    vehicle.fixed_frames = {{.name = "chain_root_mount", .body_name = "chain_root"},
                            {.name = "chain_middle_mount", .body_name = "chain_middle"},
                            {.name = "lone_mount", .body_name = "lone"}};
    vehicle.revolute_joints = {
        {.name = "chain_pivot",
         .parent_frame_name = "chain_root_mount",
         .child_frame_name = "chain_middle"},
        {.name = "loop_a", .parent_frame_name = "loop_root", .child_frame_name = "loop_first"},
        {.name = "loop_b", .parent_frame_name = "loop_first", .child_frame_name = "loop_second"},
        {.name = "loop_c", .parent_frame_name = "loop_second", .child_frame_name = "loop_root"}};
    vehicle.ball_rpy_joints = {{.name = "dangling_ball",
                                .parent_frame_name = "no_such_frame",
                                .child_frame_name = "lone"}};
    vehicle.weld_joints = {
        {.name = "reversed_weld",
         .parent_frame_name = "chain_end",
         .child_frame_name = "chain_middle_mount"},
        {.name = "self_weld", .parent_frame_name = "lone", .child_frame_name = "lone_mount"},
        {.name = "pair_weld", .parent_frame_name = "pair_first", .child_frame_name = "pair_second"},
        {.name = "first_weld", .parent_frame_name = "first_free", .child_frame_name = "between"},
        {.name = "second_weld", .parent_frame_name = "second_free", .child_frame_name = "between"}};
    return project;
}

void VerifyRecordsAProjectFileCanHold() {
    const ContractTexts texts;
    const VehicleProject project = ProjectOfSeveralGroups();
    const auto read = [&project](const std::string& body) {
        return ReadRigidBodyAttachment(project, body);
    };
    const auto frames_outside = [&project](const std::set<std::string>& left_out) {
        return FramesOutside(project, left_out);
    };

    const RigidBodyAttachment root = read("chain_root");
    Require(root.state == RigidBodyAttachmentState::kFree && root.summary == texts.free &&
                !root.note.has_value() && !root.held_at.has_value() && root.can_take_joint &&
                !root.can_become_free,
            "the free root of a chain does not read free, able to take a joint");
    Require(root.frames_on_body ==
                    std::vector<ElementReference>{{ElementKind::kRigidBody, "chain_root"},
                                                  {ElementKind::kFixedFrame, "chain_root_mount"}} &&
                root.frames_it_can_be_held_at ==
                    frames_outside({"chain_root", "chain_middle", "chain_end"}),
            "the root of a chain may be held at a frame of its own chain");

    const ElementReference pivot{ElementKind::kRevoluteJoint, "chain_pivot"};
    const RigidBodyAttachment middle = read("chain_middle");
    Require(middle.state == RigidBodyAttachmentState::kHangsOnJoint && middle.joint == pivot &&
                middle.summary == texts.hanging &&
                middle.held_at == ElementReference{ElementKind::kFixedFrame, "chain_root_mount"} &&
                middle.can_become_free && middle.can_take_joint &&
                middle.frames_it_can_be_held_at == frames_outside({"chain_middle", "chain_end"}),
            "a body hanging below a free one does not hang on its joint, or may be held at a "
            "frame of the body below it");

    // The record's parent end is on the hanging body: it hangs all the same.
    const ElementReference reversed{ElementKind::kWeldJoint, "reversed_weld"};
    const RigidBodyAttachment end = read("chain_end");
    Require(end.state == RigidBodyAttachmentState::kHangsOnJoint && end.joint == reversed &&
                end.summary == texts.hanging &&
                end.held_at == ElementReference{ElementKind::kFixedFrame, "chain_middle_mount"} &&
                end.frames_it_can_be_held_at == frames_outside({"chain_end"}),
            "a body on the parent end of the joint it hangs on does not read as hanging on it, "
            "held at the joint's child end");

    // A joint naming no frame, and a joint with both ends on one body, relate
    // nothing.
    const RigidBodyAttachment lone = read("lone");
    Require(lone.state == RigidBodyAttachmentState::kUnattached &&
                lone.summary == texts.not_attached && lone.note == texts.not_attached_note &&
                !lone.joint.has_value() && !lone.held_at.has_value() && lone.can_become_free &&
                lone.can_take_joint && lone.frames_it_can_be_held_at == frames_outside({"lone"}),
            "a lone body not stated free, with a joint naming no frame and one on itself, does "
            "not read unattached");

    for (const std::string body : {"pair_first", "pair_second"}) {
        const RigidBodyAttachment pair = read(body);
        Require(pair.state == RigidBodyAttachmentState::kUnattached &&
                    pair.summary == texts.not_attached && pair.note == texts.not_attached_note &&
                    !pair.joint.has_value() && !pair.held_at.has_value() && pair.can_become_free &&
                    pair.can_take_joint &&
                    pair.frames_it_can_be_held_at == frames_outside({"pair_first", "pair_second"}),
                "rigid body '" + body +
                    "', in a group without a free body, does not read unattached, held "
                    "nowhere in its group");
    }

    const auto require_conflicting = [&read, &texts](const std::string& body, const Title& note) {
        const RigidBodyAttachment conflicting = read(body);
        Require(conflicting.state == RigidBodyAttachmentState::kConflicting &&
                    conflicting.summary == texts.unclear && conflicting.note == note &&
                    !conflicting.joint.has_value() && !conflicting.held_at.has_value() &&
                    !conflicting.can_become_free && !conflicting.can_take_joint,
                "rigid body '" + body +
                    "' does not read as conflicting, saying why and able to change nothing");
    };
    for (const std::string body : {"loop_root", "loop_first", "loop_second"}) {
        require_conflicting(body, texts.loop_note);
    }
    for (const std::string body : {"first_free", "between", "second_free"}) {
        require_conflicting(body, texts.several_free_bodies_note);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_rigid_body_attachment_reading GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const ReferenceVehicle irw =
            editing_verification::LoadReferenceVehicle(argv[4], argv[5], argv[6]);
        VerifyBundledVehicle(PlacedProjectWithContact(gz18));
        VerifyBundledVehicle(PlacedProjectWithContact(irw));
        VerifyRecordsAProjectFileCanHold();
        std::puts(
            "vehicle project rigid body attachment reading passed for GZ18, IRW and records "
            "a project file can hold");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project rigid body attachment reading: %s\n", error.what());
        return 1;
    }
}
