#include <Eigen/Geometry>
#include <algorithm>
#include <cstdio>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "configuration_record_comparison.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_contact_topology.h"
#include "orvd/vehicle_project/free_body_placements.h"
#include "orvd/vehicle_project/generate_assembly_inputs.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_invariants.h"

// For both bundled vehicles: a project holding the vehicle definition
// generates that definition again, field by field; with the reference contact
// record the wheel-pair stations come from the contact carriers; the reference
// start-up poses place every free body, and a start-up state bound to another
// vehicle is refused; nesting, moving and renaming groups changes neither
// generated record; and inconsistent placements and groups are refused with a
// message naming the offender.

namespace {

using namespace orvd;
using configuration_record_comparison::Same;
using vehicle_project::ElementGroup;
using vehicle_project::ElementKind;
using vehicle_project::VehicleProject;

[[noreturn]] void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

void Require(bool condition, const std::string& detail) {
    if (!condition) {
        Fail(detail);
    }
}

// `action` must throw std::invalid_argument with a message containing
// `offender`.
template <typename Action>
void RequireRefusal(Action action, const std::string& offender, const std::string& what) {
    try {
        action();
    } catch (const std::invalid_argument& error) {
        Require(std::string_view(error.what()).find(offender) != std::string_view::npos,
                what + " was refused without naming " + offender + ": " + error.what());
        return;
    }
    Fail(what + " was accepted");
}

struct ReferenceVehicle {
    configuration::VehicleDefinition definition;
    configuration::ResolvedStartupState startup_state;
    configuration::VehicleContactRecord contact_record;
};

void VerifyImportAndGeneration(const ReferenceVehicle& reference) {
    const std::string& vehicle_name = reference.definition.vehicle_name;
    const VehicleProject project{.vehicle_definition = reference.definition};
    Require(!project.contact_record && project.free_body_placements.empty() &&
                project.element_groups.empty(),
            vehicle_name + ": an imported project holds more than its vehicle definition");
    vehicle_project::RequireVehicleProjectInvariants(project);
    Require(Same(reference.definition, vehicle_project::GenerateVehicleDefinition(project)),
            vehicle_name +
                ": the definition generated from the imported project differs from "
                "the loaded one");

    // The layout places every free body exactly once, so it lists the bodies
    // an imported project has not placed.
    std::vector<std::string> free_body_names;
    for (const configuration::VehicleFreeBodyStationOffsetDefinition& offset :
         reference.definition.mechanical_track_station_layout.free_body_station_offsets) {
        free_body_names.push_back(offset.body_name);
    }
    std::vector<std::string> unplaced = vehicle_project::CollectUnplacedFreeBodyNames(project);
    std::ranges::sort(free_body_names);
    std::ranges::sort(unplaced);
    Require(unplaced == free_body_names,
            vehicle_name + ": the imported project does not report every free body as unplaced");

    RequireRefusal([&project] { (void)vehicle_project::GenerateVehicleContactRecord(project); },
                   "wheel-rail contact record",
                   vehicle_name + ": generating the contact record of a project without one");
}

void VerifyContactCarriersProvideStations(const ReferenceVehicle& reference) {
    const std::string& vehicle_name = reference.definition.vehicle_name;
    VehicleProject project{.vehicle_definition = reference.definition};
    project.contact_record = reference.contact_record;
    // With the imported list emptied, the stations can only come from the
    // carriers.
    project.vehicle_definition.mechanical_track_station_layout
        .wheel_pair_station_reference_body_names.clear();
    const configuration::VehicleDefinition generated =
        vehicle_project::GenerateVehicleDefinition(project);
    Require(generated.mechanical_track_station_layout.wheel_pair_station_reference_body_names ==
                reference.definition.mechanical_track_station_layout
                    .wheel_pair_station_reference_body_names,
            vehicle_name +
                ": the wheel-pair stations generated from the contact carriers differ "
                "from the list the vehicle file states");
    Require(Same(reference.definition, generated),
            vehicle_name +
                ": generating with a contact record changed more than the wheel-pair "
                "stations");
    Require(Same(reference.contact_record, vehicle_project::GenerateVehicleContactRecord(project)),
            vehicle_name + ": the generated contact record differs from the project's");
}

void RequirePlacedAtStartupPoses(const VehicleProject& project, const ReferenceVehicle& reference) {
    const std::string& vehicle_name = reference.definition.vehicle_name;
    for (const configuration::VehicleRigidBodyDefinition& body :
         reference.definition.rigid_bodies) {
        if (!body.moves_freely_in_world) {
            continue;
        }
        const auto state =
            std::ranges::find(reference.startup_state.free_body_startup_states, body.name,
                              &configuration::FreeBodyStartupState::body_name);
        const auto placement = std::ranges::find(project.free_body_placements, body.name,
                                                 &vehicle_project::FreeBodyPlacement::body_name);
        Require(state != reference.startup_state.free_body_startup_states.end(),
                vehicle_name + ": the reference start-up state has no pose for free body '" +
                    body.name + "'");
        Require(
            placement != project.free_body_placements.end() &&
                placement->rotation_local_track_from_body.coeffs() ==
                    state->rotation_local_track_from_body.coeffs() &&
                placement->lateral_offset_in_local_track_frame_meters ==
                    state->lateral_offset_in_local_track_frame_meters &&
                placement->vertical_offset_in_local_track_frame_meters ==
                    state->vertical_offset_in_local_track_frame_meters,
            vehicle_name + ": free body '" + body.name + "' is not placed at its start-up pose");
    }
}

void VerifyStartupPosesPlaceEveryFreeBody(const VehicleProject& placed,
                                          const ReferenceVehicle& reference) {
    const std::string& vehicle_name = reference.definition.vehicle_name;
    RequirePlacedAtStartupPoses(placed, reference);
    Require(vehicle_project::CollectUnplacedFreeBodyNames(placed).empty(),
            vehicle_name + ": a free body is unplaced after placing at the start-up poses");
    vehicle_project::RequireVehicleProjectInvariants(placed);
    Require(Same(reference.definition, vehicle_project::GenerateVehicleDefinition(placed)),
            vehicle_name + ": placements changed the generated definition");

    VehicleProject project = placed;
    const std::string removed_body_name = project.free_body_placements.back().body_name;
    project.free_body_placements.pop_back();
    Require(vehicle_project::CollectUnplacedFreeBodyNames(project) ==
                std::vector<std::string>{removed_body_name},
            vehicle_name + ": removing the placement of '" + removed_body_name +
                "' did not leave exactly that body unplaced");
}

void VerifyPlacementRefusals(const VehicleProject& placed) {
    const std::string& vehicle_name = placed.vehicle_definition.vehicle_name;
    VehicleProject project = placed;
    const auto check_project = [&project] {
        vehicle_project::RequireVehicleProjectInvariants(project);
    };

    const auto jointed_body =
        std::ranges::find(placed.vehicle_definition.rigid_bodies, false,
                          &configuration::VehicleRigidBodyDefinition::moves_freely_in_world);
    Require(jointed_body != placed.vehicle_definition.rigid_bodies.end(),
            vehicle_name + " has no body that is not free");
    project.free_body_placements.push_back({.body_name = jointed_body->name});
    RequireRefusal(check_project, "free_body_placements['" + jointed_body->name + "']",
                   vehicle_name + ": a placement of a body that does not move freely");

    const vehicle_project::FreeBodyPlacement& first = placed.free_body_placements.front();
    project = placed;
    project.free_body_placements.push_back(first);
    RequireRefusal(check_project, "repeats '" + first.body_name + "'",
                   vehicle_name + ": a second placement of one body");

    project = placed;
    project.free_body_placements.front().lateral_offset_in_local_track_frame_meters =
        std::numeric_limits<double>::quiet_NaN();
    RequireRefusal(check_project,
                   "free_body_placements['" + first.body_name +
                       "'].lateral_offset_in_local_track_frame_meters",
                   vehicle_name + ": a lateral offset that is not finite");

    project = placed;
    project.free_body_placements.front().rotation_local_track_from_body.coeffs() *= 2.0;
    RequireRefusal(check_project,
                   "free_body_placements['" + first.body_name + "'].rotation_local_track_from_body",
                   vehicle_name + ": a rotation that is not a unit quaternion");
}

// A carrier and its body share a name; as members of different kinds they are
// different elements and may sit in different groups.
std::vector<ElementGroup> MakeElementGroups(const VehicleProject& project) {
    const configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    const configuration::VehicleContactDeclaration& declaration =
        project.contact_record->declaration;
    return {
        {.name = "running gear",
         .members = {{ElementKind::kContactCarrier, declaration.carriers.front().carrier_name}},
         .subgroups = {{.name = "first wheel pair",
                        .members = {{ElementKind::kRigidBody,
                                     declaration.carriers.front().body_name},
                                    {ElementKind::kContactInterface,
                                     declaration.interfaces.front().interface_name}}}}},
        {.name = "suspension",
         .members = {{ElementKind::kTranslationalSpringDamper,
                      vehicle.translational_spring_dampers.front().name},
                     {ElementKind::kRevoluteJoint, vehicle.revolute_joints.front().name}}},
        {.name = "attachments",
         .members = {{ElementKind::kFixedFrame, vehicle.fixed_frames.front().name}}},
    };
}

void VerifyGroupingOnlyOrganises(VehicleProject project) {
    const std::string& vehicle_name = project.vehicle_definition.vehicle_name;
    const configuration::VehicleDefinition definition =
        vehicle_project::GenerateVehicleDefinition(project);
    const configuration::VehicleContactRecord contact_record =
        vehicle_project::GenerateVehicleContactRecord(project);
    const auto require_generation_unchanged = [&](const std::string& step) {
        vehicle_project::RequireVehicleProjectInvariants(project);
        Require(Same(definition, vehicle_project::GenerateVehicleDefinition(project)) &&
                    Same(contact_record, vehicle_project::GenerateVehicleContactRecord(project)),
                vehicle_name + ": " + step + " changed a generated record");
    };

    project.element_groups = MakeElementGroups(project);
    require_generation_unchanged("grouping");

    // "attachments" becomes a subgroup of "suspension".
    project.element_groups[1].subgroups.push_back(std::move(project.element_groups[2]));
    project.element_groups.pop_back();
    require_generation_unchanged("nesting a group");

    // The revolute joint moves from "suspension" into the nested group.
    ElementGroup& suspension = project.element_groups[1];
    suspension.subgroups[0].members.push_back(suspension.members.back());
    suspension.members.pop_back();
    require_generation_unchanged("moving a member");

    project.element_groups[0].name = "bogies";
    require_generation_unchanged("renaming a group");
}

void VerifyGroupRefusals(const VehicleProject& grouped) {
    const std::string& vehicle_name = grouped.vehicle_definition.vehicle_name;
    const configuration::VehicleContactCarrierDeclaration& carrier =
        grouped.contact_record->declaration.carriers.front();
    VehicleProject project = grouped;
    const auto check_project = [&project] {
        vehicle_project::RequireVehicleProjectInvariants(project);
    };

    project.element_groups[1].members.push_back({ElementKind::kRigidBody, "absent_body"});
    RequireRefusal(check_project, "rigid body 'absent_body'",
                   vehicle_name + ": a member naming a missing element");

    project = grouped;
    project.element_groups[1].members.push_back({ElementKind::kFixedFrame, carrier.body_name});
    RequireRefusal(check_project, "fixed frame '" + carrier.body_name + "'",
                   vehicle_name + ": a member naming a body as a fixed frame");

    project = grouped;
    project.contact_record.reset();
    RequireRefusal(check_project, "contact carrier '" + carrier.carrier_name + "'",
                   vehicle_name + ": a contact carrier as a member of a project without contact");

    project = grouped;
    project.element_groups[1].members.push_back({ElementKind::kRigidBody, carrier.body_name});
    RequireRefusal(check_project,
                   "rigid body '" + carrier.body_name +
                       "', which element_groups['running gear'].subgroups['first wheel pair'] "
                       "already contains",
                   vehicle_name + ": an element in two groups");

    project = grouped;
    project.element_groups[2].members.push_back(grouped.element_groups[2].members.front());
    RequireRefusal(check_project, "which element_groups['attachments'] already contains",
                   vehicle_name + ": an element twice in one group");

    project = grouped;
    project.element_groups[0].subgroups.push_back({.name = "first wheel pair"});
    RequireRefusal(check_project, "repeats the group name 'first wheel pair'",
                   vehicle_name + ": two sibling groups of one name");

    project = grouped;
    project.element_groups.push_back({.name = ""});
    RequireRefusal(check_project, "element_groups[3].name is empty",
                   vehicle_name + ": a group without a name");
}

void VerifyReferenceVehicle(const ReferenceVehicle& reference) {
    VerifyImportAndGeneration(reference);
    VerifyContactCarriersProvideStations(reference);

    VehicleProject project{.vehicle_definition = reference.definition};
    vehicle_project::PlaceFreeBodiesAtStartupPoses(project, reference.startup_state);
    VerifyStartupPosesPlaceEveryFreeBody(project, reference);
    VerifyPlacementRefusals(project);

    project.contact_record = reference.contact_record;
    VerifyGroupingOnlyOrganises(project);
    project.element_groups = MakeElementGroups(project);
    VerifyGroupRefusals(project);
}

void VerifyStartupStateBinding(const ReferenceVehicle& gz18, const ReferenceVehicle& irw) {
    VehicleProject project{.vehicle_definition = irw.definition};
    vehicle_project::PlaceFreeBodiesAtStartupPoses(project, irw.startup_state);
    RequireRefusal(
        [&] { vehicle_project::PlaceFreeBodiesAtStartupPoses(project, gz18.startup_state); },
        "'" + gz18.startup_state.vehicle_binding.vehicle_name + "'",
        "placing the IRW project at the GZ18 start-up poses");
    RequirePlacedAtStartupPoses(project, irw);

    // The mechanical definition identifier binds as well as the vehicle name.
    configuration::ResolvedStartupState other_mechanics = irw.startup_state;
    other_mechanics.vehicle_binding.mechanical_definition_identifier = "edited_irw_mechanics";
    RequireRefusal(
        [&] { vehicle_project::PlaceFreeBodiesAtStartupPoses(project, other_mechanics); },
        "'edited_irw_mechanics'",
        "placing the IRW project at poses resolved for another mechanical definition");
    RequirePlacedAtStartupPoses(project, irw);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project GZ18_VEHICLE GZ18_STARTUP_STATE GZ18_CONTACT "
                     "IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT\n");
        return 2;
    }
    try {
        const configuration::VehicleDefinition gz18_definition =
            configuration::LoadVehicleDefinitionFromJsonFile(argv[1]);
        const ReferenceVehicle gz18{gz18_definition,
                                    configuration::LoadResolvedStartupStateFromJsonFile(argv[2]),
                                    configuration::LoadVehicleContactRecordFromJsonFile(argv[3])};
        const configuration::VehicleDefinition irw_definition =
            configuration::LoadVehicleDefinitionFromJsonFile(argv[4]);
        const ReferenceVehicle irw{irw_definition,
                                   configuration::LoadResolvedStartupStateFromJsonFile(argv[5]),
                                   configuration::LoadVehicleContactRecordFromJsonFile(argv[6])};

        (void)configuration::BuildVehicleContactTopology(gz18.definition,
                                                         gz18.contact_record.declaration);
        (void)configuration::BuildVehicleContactTopology(irw.definition,
                                                         irw.contact_record.declaration);
        VerifyReferenceVehicle(gz18);
        VerifyReferenceVehicle(irw);
        VerifyStartupStateBinding(gz18, irw);
        std::puts(
            "vehicle project import, generation, start-up placement, grouping and "
            "refusals passed for GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project: %s\n", error.what());
        return 1;
    }
}
