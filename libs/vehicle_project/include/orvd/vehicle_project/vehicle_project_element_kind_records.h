#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <memory>
#include <optional>
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
// name, how a diagnostic names the kind, the word a project file and the
// modelling service write the kind as, and the project's container of those
// records. VisitElementKind turns a kind known only at run time into that
// compile-time statement; it needs no project, so it serves questions about a
// kind as well as questions about a project's elements. FindElementRecord and
// ContainsElement look one element up by its name, DescribeElementKind and
// DescribeElement name a kind and an element in diagnostics, and
// ElementKindWord and FindElementKindWrittenAs turn a kind into its word and
// back.

namespace orvd::vehicle_project {

// Every element kind, in the order ElementKind declares them, which is the
// order an outline lists elements in, kind by kind.
inline constexpr std::array kElementKinds{
    ElementKind::kRigidBody,
    ElementKind::kFixedFrame,
    ElementKind::kRevoluteJoint,
    ElementKind::kBallRpyJoint,
    ElementKind::kWeldJoint,
    ElementKind::kTranslationalSpringDamper,
    ElementKind::kRollSpringDamperCouple,
    ElementKind::kSeriesSpringViscousDamper,
    ElementKind::kPointToPointSeriesSpringBilinearDamper,
    ElementKind::kSaturatedPiecewiseLinearDamper,
    ElementKind::kReferenceFrameLinearBushing,
    ElementKind::kPointToPointLinearSpringDamper,
    ElementKind::kPointToPointHystereticSpring,
    ElementKind::kContactCarrier,
    ElementKind::kContactInterface,
};
static_assert(
    [] {
        for (std::size_t index = 0; index < kElementKinds.size(); ++index) {
            if (static_cast<std::size_t>(std::to_underlying(kElementKinds[index])) != index) {
                return false;
            }
        }
        return true;
    }(),
    "kElementKinds lists the element kinds in declaration order, each once");

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
    static constexpr std::string_view kKindWord = "rigid_body";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.rigid_bodies;
    }
};

template <>
struct ElementKindRecords<ElementKind::kFixedFrame> {
    using Record = configuration::VehicleFixedFrameDefinition;
    static constexpr std::string_view kKindDescription = "fixed frame";
    static constexpr std::string_view kKindWord = "fixed_frame";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.fixed_frames;
    }
};

template <>
struct ElementKindRecords<ElementKind::kRevoluteJoint> {
    using Record = configuration::VehicleRevoluteJointDefinition;
    static constexpr std::string_view kKindDescription = "revolute joint";
    static constexpr std::string_view kKindWord = "revolute_joint";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.revolute_joints;
    }
};

template <>
struct ElementKindRecords<ElementKind::kBallRpyJoint> {
    using Record = configuration::VehicleBallRpyJointDefinition;
    static constexpr std::string_view kKindDescription = "Ball-RPY joint";
    static constexpr std::string_view kKindWord = "ball_rpy_joint";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.ball_rpy_joints;
    }
};

template <>
struct ElementKindRecords<ElementKind::kWeldJoint> {
    using Record = configuration::VehicleWeldJointDefinition;
    static constexpr std::string_view kKindDescription = "weld joint";
    static constexpr std::string_view kKindWord = "weld_joint";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.weld_joints;
    }
};

template <>
struct ElementKindRecords<ElementKind::kTranslationalSpringDamper> {
    using Record = configuration::VehicleTranslationalSpringDamperDefinition;
    static constexpr std::string_view kKindDescription = "translational spring-damper";
    static constexpr std::string_view kKindWord = "translational_spring_damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.translational_spring_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kRollSpringDamperCouple> {
    using Record = configuration::VehicleRollSpringDamperCoupleDefinition;
    static constexpr std::string_view kKindDescription = "roll spring-damper couple";
    static constexpr std::string_view kKindWord = "roll_spring_damper_couple";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.roll_spring_damper_couples;
    }
};

template <>
struct ElementKindRecords<ElementKind::kSeriesSpringViscousDamper> {
    using Record = configuration::VehicleSeriesSpringViscousDamperDefinition;
    static constexpr std::string_view kKindDescription = "series spring-viscous damper";
    static constexpr std::string_view kKindWord = "series_spring_viscous_damper";
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
    static constexpr std::string_view kKindWord = "point_to_point_series_spring_bilinear_damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.point_to_point_series_spring_bilinear_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kSaturatedPiecewiseLinearDamper> {
    using Record = configuration::VehicleSaturatedPiecewiseLinearDamperDefinition;
    static constexpr std::string_view kKindDescription = "saturated piecewise-linear damper";
    static constexpr std::string_view kKindWord = "saturated_piecewise_linear_damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.saturated_piecewise_linear_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kReferenceFrameLinearBushing> {
    using Record = configuration::VehicleReferenceFrameLinearBushingDefinition;
    static constexpr std::string_view kKindDescription = "reference-frame linear bushing";
    static constexpr std::string_view kKindWord = "reference_frame_linear_bushing";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.reference_frame_linear_bushings;
    }
};

template <>
struct ElementKindRecords<ElementKind::kPointToPointLinearSpringDamper> {
    using Record = configuration::PointToPointLinearSpringDamperDefinition;
    static constexpr std::string_view kKindDescription = "point-to-point linear spring-damper";
    static constexpr std::string_view kKindWord = "point_to_point_linear_spring_damper";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.point_to_point_linear_spring_dampers;
    }
};

template <>
struct ElementKindRecords<ElementKind::kPointToPointHystereticSpring> {
    using Record = configuration::PointToPointHystereticSpringDefinition;
    static constexpr std::string_view kKindDescription = "point-to-point hysteretic spring";
    static constexpr std::string_view kKindWord = "point_to_point_hysteretic_spring";
    static constexpr std::string Record::* kElementName = &Record::name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return &project.vehicle_definition.point_to_point_hysteretic_springs;
    }
};

template <>
struct ElementKindRecords<ElementKind::kContactCarrier> {
    using Record = configuration::VehicleContactCarrierDeclaration;
    static constexpr std::string_view kKindDescription = "contact carrier";
    static constexpr std::string_view kKindWord = "contact_carrier";
    static constexpr std::string Record::* kElementName = &Record::carrier_name;
    static auto* Records(ConstOrMutableVehicleProject auto& project) {
        return project.contact_record ? &project.contact_record->declaration.carriers : nullptr;
    }
};

template <>
struct ElementKindRecords<ElementKind::kContactInterface> {
    using Record = configuration::VehicleContactInterfaceDeclaration;
    static constexpr std::string_view kKindDescription = "contact interface";
    static constexpr std::string_view kKindWord = "contact_interface";
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

// The record of the element of KindRecords' kind named `name`, sharing the
// project's constness, or null when the project has none; a contact kind has
// none while the project has no contact record.
template <typename KindRecords, ConstOrMutableVehicleProject Project>
auto* FindElementRecord(Project& project, const std::string& name) {
    auto* const records = KindRecords::Records(project);
    using RecordPointer = decltype(records->data());
    if (records == nullptr) {
        return RecordPointer{nullptr};
    }
    const auto found = std::ranges::find(*records, name, KindRecords::kElementName);
    return found == records->end() ? RecordPointer{nullptr} : std::to_address(found);
}

// Whether the project contains `element`.
inline bool ContainsElement(const VehicleProject& project, const ElementReference& element) {
    return VisitElementKind(element.kind, [&project, &element]<typename KindRecords>(KindRecords) {
        return FindElementRecord<KindRecords>(project, element.name) != nullptr;
    });
}

// How a diagnostic names a kind: its kKindDescription, "rigid body".
inline std::string_view DescribeElementKind(ElementKind kind) {
    return VisitElementKind(
        kind, []<typename KindRecords>(KindRecords) { return KindRecords::kKindDescription; });
}

// How a diagnostic names an element: its kind and its name, "rigid body
// 'carbody'".
inline std::string DescribeElement(const ElementReference& element) {
    return std::string(DescribeElementKind(element.kind)) + " '" + element.name + "'";
}

// The word a project file and the modelling service write a kind as: its
// kKindWord, "rigid_body".
inline std::string_view ElementKindWord(ElementKind kind) {
    return VisitElementKind(
        kind, []<typename KindRecords>(KindRecords) { return KindRecords::kKindWord; });
}

// The kind written as `word`, or none when no kind is.
inline std::optional<ElementKind> FindElementKindWrittenAs(std::string_view word) {
    const auto written = std::ranges::find_if(
        kElementKinds, [word](ElementKind kind) { return ElementKindWord(kind) == word; });
    if (written == kElementKinds.end()) {
        return std::nullopt;
    }
    return *written;
}

}  // namespace orvd::vehicle_project
