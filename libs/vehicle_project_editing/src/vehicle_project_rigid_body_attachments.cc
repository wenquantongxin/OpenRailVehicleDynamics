#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <map>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_name_spaces.h"
#include "vehicle_project_element_property_tables.h"
#include "vehicle_project_records_following_elements.h"

namespace orvd::vehicle_project_editing {
namespace {

using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;

// The joint properties naming the end on the body and the other end.
constexpr std::string_view kBodyEndProperty = "child_frame_name";
constexpr std::string_view kOtherEndProperty = "parent_frame_name";

// The properties only a free body has: those a rigid body's records keyed by
// its name hold.
constexpr auto kFreeOnlyProperties = [] {
    std::array<std::string_view, internal::kBodyKeyedProperties.size()> names{};
    for (std::size_t index = 0; index < names.size(); ++index) {
        names[index] = internal::kBodyKeyedProperties[index].name;
    }
    return names;
}();

// One joint relating two bodies: the bodies its ends are on, by their place
// among the project's bodies, and the frames the ends name.
struct RelatingJoint {
    ElementReference joint;
    std::size_t parent_body;
    std::size_t child_body;
    std::string parent_frame_name;
    std::string child_frame_name;
};

// Calls `visit(joint, parent_frame_name, child_frame_name)` for every joint
// record, kind by kind in ElementKind's order, each kind in declaration order.
template <typename Visitor>
void ForEachJointRecord(const VehicleProject& project, Visitor&& visit) {
    for (const ElementKind kind : internal::kJointKinds) {
        vehicle_project::VisitElementKind(kind, [&]<typename KindRecords>(KindRecords) {
            using Record = typename KindRecords::Record;
            if constexpr (requires(const Record& joint) {
                              joint.parent_frame_name;
                              joint.child_frame_name;
                          }) {
                for (const Record& joint : *KindRecords::Records(project)) {
                    visit(ElementReference{kind, joint.*KindRecords::kElementName},
                          joint.parent_frame_name, joint.child_frame_name);
                }
            }
        });
    }
}

// The project's rigid bodies, the joints relating them and the groups they
// form, as attachments are read from them. A body is known by the first record
// of its name, as an edit finds it.
class BodiesRelatedByJoints {
  public:
    explicit BodiesRelatedByJoints(const VehicleProject& project) : project_(project) {
        for (const configuration::VehicleRigidBodyDefinition& body :
             project.vehicle_definition.rigid_bodies) {
            if (place_of_body_.emplace(body.name, bodies_.size()).second) {
                bodies_.push_back(&body);
            }
        }
        ForEachJointRecord(project, [this](const ElementReference& joint,
                                           const std::string& parent_frame_name,
                                           const std::string& child_frame_name) {
            const std::optional<std::size_t> parent_body = BodyOfFrame(parent_frame_name);
            const std::optional<std::size_t> child_body = BodyOfFrame(child_frame_name);
            if (parent_body.has_value() && child_body.has_value() && *parent_body != *child_body) {
                joints_.push_back(
                    {joint, *parent_body, *child_body, parent_frame_name, child_frame_name});
            }
        });
        group_.resize(bodies_.size());
        std::iota(group_.begin(), group_.end(), std::size_t{0});
        for (const RelatingJoint& joint : joints_) {
            const std::size_t parent_group = GroupOf(joint.parent_body);
            group_[parent_group] = GroupOf(joint.child_body);
        }
        for (std::size_t body = 0; body < bodies_.size(); ++body) {
            group_[body] = GroupOf(body);
        }
    }

    [[nodiscard]] std::optional<std::size_t> PlaceOfBody(const std::string& name) const {
        const auto found = place_of_body_.find(name);
        return found == place_of_body_.end() ? std::nullopt : std::optional(found->second);
    }

    [[nodiscard]] const configuration::VehicleRigidBodyDefinition& Body(std::size_t body) const {
        return *bodies_[body];
    }

    [[nodiscard]] std::size_t BodyCount() const { return bodies_.size(); }

    // The bodies `from` reaches through relating joints, itself included,
    // without passing `left_out`.
    [[nodiscard]] std::vector<bool> Reached(std::size_t from,
                                            const std::optional<ElementReference>& left_out) const {
        std::vector<bool> reached(bodies_.size(), false);
        std::vector<std::size_t> pending{from};
        reached[from] = true;
        while (!pending.empty()) {
            const std::size_t body = pending.back();
            pending.pop_back();
            for (const RelatingJoint& joint : joints_) {
                if (left_out.has_value() && joint.joint == *left_out) {
                    continue;
                }
                const std::optional<std::size_t> other = OtherBody(joint, body);
                if (other.has_value() && !reached[*other]) {
                    reached[*other] = true;
                    pending.push_back(*other);
                }
            }
        }
        return reached;
    }

    // How the bodies of `body`'s group are attached, the group being read as
    // a whole: `body`'s state and, for a body hanging on a joint, that joint
    // and the frame it is held at.
    struct Reading {
        RigidBodyAttachmentState state{RigidBodyAttachmentState::kUnattached};
        bool closes_loop{false};
        const RelatingJoint* hung_on{nullptr};
        std::optional<ElementReference> held_at{};
    };

    [[nodiscard]] Reading Read(std::size_t body) const {
        const std::size_t group = group_[body];
        std::size_t body_count = 0;
        std::size_t joint_count = 0;
        std::vector<std::size_t> free_bodies;
        for (std::size_t member = 0; member < bodies_.size(); ++member) {
            if (group_[member] == group) {
                ++body_count;
                if (bodies_[member]->moves_freely_in_world) {
                    free_bodies.push_back(member);
                }
            }
        }
        for (const RelatingJoint& joint : joints_) {
            joint_count += group_[joint.parent_body] == group ? 1 : 0;
        }
        if (joint_count >= body_count) {
            return {.state = RigidBodyAttachmentState::kConflicting, .closes_loop = true};
        }
        if (free_bodies.size() > 1) {
            return {.state = RigidBodyAttachmentState::kConflicting};
        }
        if (free_bodies.empty()) {
            return {.state = RigidBodyAttachmentState::kUnattached};
        }
        if (free_bodies.front() == body) {
            return {.state = RigidBodyAttachmentState::kFree};
        }
        // The path from the free body to `body` is the group's only one, the
        // group having no loop; its last joint is the one `body` hangs on.
        std::vector<const RelatingJoint*> reached_by(bodies_.size(), nullptr);
        std::vector<bool> reached(bodies_.size(), false);
        std::vector<std::size_t> pending{free_bodies.front()};
        reached[free_bodies.front()] = true;
        while (!pending.empty()) {
            const std::size_t from = pending.back();
            pending.pop_back();
            for (const RelatingJoint& joint : joints_) {
                const std::optional<std::size_t> other = OtherBody(joint, from);
                if (other.has_value() && !reached[*other]) {
                    reached[*other] = true;
                    reached_by[*other] = &joint;
                    pending.push_back(*other);
                }
            }
        }
        const RelatingJoint& joint = *reached_by[body];
        return {.state = RigidBodyAttachmentState::kHangsOnJoint,
                .hung_on = &joint,
                .held_at = FrameNamed(joint.child_body == body ? joint.parent_frame_name
                                                               : joint.child_frame_name)};
    }

  private:
    // The body a frame is on: the body of that name, or the body a fixed frame
    // of that name names; none when that is not a body of the project.
    [[nodiscard]] std::optional<std::size_t> BodyOfFrame(const std::string& frame_name) const {
        if (const std::optional<std::size_t> body = PlaceOfBody(frame_name)) {
            return body;
        }
        const auto& frames = project_.vehicle_definition.fixed_frames;
        const auto frame = std::ranges::find(frames, frame_name,
                                             &configuration::VehicleFixedFrameDefinition::name);
        return frame == frames.end() ? std::nullopt : PlaceOfBody(frame->body_name);
    }

    // The frame a relating joint's end names: the body of that name, or else
    // the fixed frame of that name.
    [[nodiscard]] ElementReference FrameNamed(const std::string& frame_name) const {
        return {PlaceOfBody(frame_name).has_value() ? ElementKind::kRigidBody
                                                    : ElementKind::kFixedFrame,
                frame_name};
    }

    // The body at the other end of `joint` from `body`, when one end is on it.
    static std::optional<std::size_t> OtherBody(const RelatingJoint& joint, std::size_t body) {
        if (joint.parent_body == body) {
            return joint.child_body;
        }
        if (joint.child_body == body) {
            return joint.parent_body;
        }
        return std::nullopt;
    }

    std::size_t GroupOf(std::size_t body) {
        while (group_[body] != body) {
            group_[body] = group_[group_[body]];
            body = group_[body];
        }
        return body;
    }

    const VehicleProject& project_;
    std::map<std::string, std::size_t> place_of_body_;
    std::vector<const configuration::VehicleRigidBodyDefinition*> bodies_;
    std::vector<RelatingJoint> joints_;
    std::vector<std::size_t> group_;
};

// `body`'s own frame and its fixed frames.
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

// What is the matter with a conflicting body.
Title DescribeConflict(bool closes_loop) {
    if (closes_loop) {
        return {.en = "The joints around it close a loop; remove one of them",
                .zh = "它周围的铰连成了环，请删除其中一个"};
    }
    return {.en =
                "It is joined through joints to more than one free body; remove one of the "
                "joints between them",
            .zh = "它经由铰连到了不止一个自由刚体，请删除其间的一个铰"};
}

}  // namespace

RigidBodyAttachment ReadRigidBodyAttachment(const VehicleProject& project,
                                            const std::string& body_name) {
    const BodiesRelatedByJoints bodies(project);
    const std::optional<std::size_t> body = bodies.PlaceOfBody(body_name);
    if (!body.has_value()) {
        throw std::invalid_argument(
            "this project contains no " +
            vehicle_project::DescribeElement({ElementKind::kRigidBody, body_name}));
    }
    const BodiesRelatedByJoints::Reading reading = bodies.Read(*body);
    RigidBodyAttachment attachment{.state = reading.state};
    switch (reading.state) {
        case RigidBodyAttachmentState::kFree:
            attachment.summary = {.en = "Moves freely in the world", .zh = "相对世界自由"};
            attachment.can_take_joint =
                internal::RequireLayoutKeepsReferenceBody(project, body_name, "attached by a joint")
                    .has_value();
            if (!attachment.can_take_joint) {
                attachment.note = Title{.en =
                                            "The station layout is measured from this body, so "
                                            "it keeps moving freely",
                                        .zh = "站位布局以它为基准，它须保持自由"};
            }
            break;
        case RigidBodyAttachmentState::kHangsOnJoint:
            attachment.summary = {.en = "Hangs on a joint", .zh = "由铰挂接"};
            attachment.joint = reading.hung_on->joint;
            attachment.held_at = reading.held_at;
            attachment.can_become_free = true;
            attachment.can_take_joint = true;
            break;
        case RigidBodyAttachmentState::kUnattached:
            attachment.summary = {.en = "Not attached", .zh = "未接入"};
            attachment.note = Title{.en = "Neither free nor held by a joint",
                                    .zh = "既没有声明自由，也没有铰把它接到别的刚体上"};
            attachment.can_become_free = true;
            attachment.can_take_joint = true;
            break;
        case RigidBodyAttachmentState::kConflicting:
            attachment.summary = {.en = "Attachment unclear", .zh = "挂接不明"};
            attachment.note = DescribeConflict(reading.closes_loop);
            break;
    }
    attachment.frames_on_body = FramesOn(project, body_name);
    const std::vector<bool> reached = bodies.Reached(*body, attachment.joint);
    for (std::size_t other = 0; other < bodies.BodyCount(); ++other) {
        if (!reached[other]) {
            std::ranges::move(FramesOn(project, bodies.Body(other).name),
                              std::back_inserter(attachment.frames_it_can_be_held_at));
        }
    }
    return attachment;
}

RigidBodyAttachmentDescription DescribeRigidBodyAttachment() {
    return {.title = {.en = "Attachment", .zh = "挂接"},
            .free_title = {.en = "Free in the world", .zh = "相对世界自由"},
            .joint_title = {.en = "Joint", .zh = "铰"},
            .held_at_title = {.en = "Held at", .zh = "挂接在"},
            .joint_kinds = internal::kJointKinds,
            .body_end_property = kBodyEndProperty,
            .other_end_property = kOtherEndProperty,
            .free_only_properties = kFreeOnlyProperties};
}

}  // namespace orvd::vehicle_project_editing
