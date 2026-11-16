#pragma once

#include <concepts>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "orvd/configuration/point_to_point_spring_definitions.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"

// What each element kind is and where a project keeps its records.
//
// For every ElementKind, ElementKindRecords<kind> states the record type that
// declares an element of that kind, the record member holding the element's
// name, how a diagnostic names the kind, and the project's container of those
// records. VisitElementKind turns a kind known only at run time into that
// compile-time statement; it needs no project, so it serves questions about a
// kind as well as questions about a project's elements.

namespace orvd::vehicle_project {

// A vehicle project seen as const or as mutable. Records reached through it
// share its constness.
template <typename Project>
concept ConstOrMutableVehicleProject = std::same_as<std::remove_const_t<Project>, VehicleProject>;

template <ElementKind kind>
struct ElementKindRecords;

// Each Records() returns a pointer to the project's container of the kind's
// records. It is never null for a kind the vehicle definition declares; for a
// contact kind it is null while the project has no contact record.

template <>
struct ElementKindRecords<ElementKind::kRigidBody> {
    using Record = configuration::VehicleRigidBodyDefinition;
    static constexpr std::string_view kKindDescription = "rigid body";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.rigid_bodies;
    }
};

template <>
struct ElementKindRecords<ElementKind::kFixedFrame> {
    using Record = configuration::VehicleFixedFrameDefinition;
    static constexpr std::string_view kKindDescription = "fixed frame";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.fixed_frames;
    }
};

template <>
struct ElementKindRecords<ElementKind::kRevoluteJoint> {
    using Record = configuration::VehicleRevoluteJointDefinition;
    static constexpr std::string_view kKindDescription = "revolute joint";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.revolute_joints;
    }
};

template <>
struct ElementKindRecords<ElementKind::kBallRpyJoint> {
    using Record = configuration::VehicleBallRpyJointDefinition;
    static constexpr std::string_view kKindDescription = "Ball-RPY joint";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.ball_rpy_joints;
    }
};

template <>
struct ElementKindRecords<ElementKind::kWeldJoint> {
    using Record = configuration::VehicleWeldJointDefinition;
    static constexpr std::string_view kKindDescription = "weld joint";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.weld_joints;
    }
};

template <>
struct ElementKindRecords<ElementKind::kTranslationalSpringDamper> {
    using Record = configuration::VehicleTranslationalSpringDamperDefinition;
    static constexpr std::string_view kKindDescription = "translational spring-damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.translational_spring_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kRollSpringDamperCouple> {
    using Record = configuration::VehicleRollSpringDamperCoupleDefinition;
    static constexpr std::string_view kKindDescription = "roll spring-damper couple";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.roll_spring_damper_couples;
    }
};

template <>
struct ElementKindRecords<ElementKind::kSeriesSpringViscousDamper> {
    using Record = configuration::VehicleSeriesSpringViscousDamperDefinition;
    static constexpr std::string_view kKindDescription = "series spring-viscous damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.series_spring_viscous_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kPointToPointSeriesSpringBilinearDamper> {
    using Record = configuration::VehiclePointToPointSeriesSpringBilinearDamperDefinition;
    static constexpr std::string_view kKindDescription =
        "point-to-point series spring-bilinear damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.point_to_point_series_spring_bilinear_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kSaturatedPiecewiseLinearDamper> {
    using Record = configuration::VehicleSaturatedPiecewiseLinearDamperDefinition;
    static constexpr std::string_view kKindDescription = "saturated piecewise-linear damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.saturated_piecewise_linear_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kReferenceFrameLinearBushing> {
    using Record = configuration::VehicleReferenceFrameLinearBushingDefinition;
    static constexpr std::string_view kKindDescription = "reference-frame linear bushing";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.reference_frame_linear_bushings;
    }
};

template <>
struct ElementKindRecords<ElementKind::kPointToPointLinearSpringDamper> {
    using Record = configuration::PointToPointLinearSpringDamperDefinition;
    static constexpr std::string_view kKindDescription = "point-to-point linear spring-damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.point_to_point_linear_spring_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kPointToPointHystereticSpring> {
    using Record = configuration::PointToPointHystereticSpringDefinition;
    static constexpr std::string_view kKindDescription = "point-to-point hysteretic spring";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.point_to_point_hysteretic_springs;
    }
};

template <>
struct ElementKindRecords<ElementKind::kContactCarrier> {
    using Record = configuration::VehicleContactCarrierDeclaration;
    static constexpr std::string_view kKindDescription = "contact carrier";
    static constexpr std::string Record::* kElementName = &Record::carrier_name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return project.contact_record ? &project.contact_record->declaration.carriers : nullptr;
    }
};

template <>
struct ElementKindRecords<ElementKind::kContactInterface> {
    using Record = configuration::VehicleContactInterfaceDeclaration;
    static constexpr std::string_view kKindDescription = "contact interface";
    static constexpr std::string Record::* kElementName = &Record::interface_name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return project.contact_record ? &project.contact_record->declaration.interfaces : nullptr;
    }
};

// Calls `visitor` with ElementKindRecords<kind>{} for the `kind` given at run
// time and returns what it returns, which must be one type for every kind.
//
// Throws std::invalid_argument when `kind` is none of ElementKind's
// enumerators, which only a cast can produce.
template <typename Visitor>
decltype(auto) VisitElementKind(ElementKind kind, Visitor&& visitor) {
    switch (kind) {
        case ElementKind::kRigidBody:
            return visitor(ElementKindRecords<ElementKind::kRigidBody>{});
        case ElementKind::kFixedFrame:
            return visitor(ElementKindRecords<ElementKind::kFixedFrame>{});
        case ElementKind::kRevoluteJoint:
            return visitor(ElementKindRecords<ElementKind::kRevoluteJoint>{});
        case ElementKind::kBallRpyJoint:
            return visitor(ElementKindRecords<ElementKind::kBallRpyJoint>{});
        case ElementKind::kWeldJoint:
            return visitor(ElementKindRecords<ElementKind::kWeldJoint>{});
        case ElementKind::kTranslationalSpringDamper:
            return visitor(ElementKindRecords<ElementKind::kTranslationalSpringDamper>{});
        case ElementKind::kRollSpringDamperCouple:
            return visitor(ElementKindRecords<ElementKind::kRollSpringDamperCouple>{});
        case ElementKind::kSeriesSpringViscousDamper:
            return visitor(ElementKindRecords<ElementKind::kSeriesSpringViscousDamper>{});
        case ElementKind::kPointToPointSeriesSpringBilinearDamper:
            return visitor(
                ElementKindRecords<ElementKind::kPointToPointSeriesSpringBilinearDamper>{});
        case ElementKind::kSaturatedPiecewiseLinearDamper:
            return visitor(ElementKindRecords<ElementKind::kSaturatedPiecewiseLinearDamper>{});
        case ElementKind::kReferenceFrameLinearBushing:
            return visitor(ElementKindRecords<ElementKind::kReferenceFrameLinearBushing>{});
        case ElementKind::kPointToPointLinearSpringDamper:
            return visitor(ElementKindRecords<ElementKind::kPointToPointLinearSpringDamper>{});
        case ElementKind::kPointToPointHystereticSpring:
            return visitor(ElementKindRecords<ElementKind::kPointToPointHystereticSpring>{});
        case ElementKind::kContactCarrier:
            return visitor(ElementKindRecords<ElementKind::kContactCarrier>{});
        case ElementKind::kContactInterface:
            return visitor(ElementKindRecords<ElementKind::kContactInterface>{});
    }
    throw std::invalid_argument("vehicle project element kind " +
                                std::to_string(std::to_underlying(kind)) +
                                " is none of the element kinds");
}

}  // namespace orvd::vehicle_project
