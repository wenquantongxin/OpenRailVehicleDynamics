#include "vehicle_project_property_edit_value_checks.h"

#include <Eigen/Core>
#include <type_traits>
#include <variant>

namespace orvd::vehicle_project_editing::internal {
namespace {

// The value kind of each type a property value holds.
constexpr PropertyValueKind ValueKindOf(std::type_identity<double>) {
    return PropertyValueKind::kNumber;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<Eigen::Vector3d>) {
    return PropertyValueKind::kVector3;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<PropertyOrientation>) {
    return PropertyValueKind::kRotation;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<PropertyText>) {
    return PropertyValueKind::kText;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<ElementOwnName>) {
    return PropertyValueKind::kElementName;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<ReferencedElementName>) {
    return PropertyValueKind::kElementReference;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<PropertyToken>) {
    return PropertyValueKind::kToken;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<SaturatedPiecewiseLinearDamperCurve>) {
    return PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve;
}

PropertyValueKind ValueKindOf(const PropertyValue& value) {
    return std::visit(
        []<typename Held>(const Held&) { return ValueKindOf(std::type_identity<Held>{}); }, value);
}

// How a refusal names a value kind.
std::string DescribeValueKind(PropertyValueKind kind) {
    switch (kind) {
        case PropertyValueKind::kNumber:
            return "a number";
        case PropertyValueKind::kVector3:
            return "a 3-vector";
        case PropertyValueKind::kRotation:
            return "a rotation";
        case PropertyValueKind::kText:
            return "a text";
        case PropertyValueKind::kElementName:
            return "an element's own name";
        case PropertyValueKind::kElementReference:
            return "a reference to an element";
        case PropertyValueKind::kToken:
            return "a token";
        case PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve:
            return "a saturated piecewise-linear damper curve";
    }
    // Reached only by a value cast into PropertyValueKind that names none of
    // its enumerators.
    return "a value of no known kind";
}

std::string Quoted(std::string_view text) {
    return "'" + std::string(text) + "'";
}

}  // namespace

std::string QuotePropertyName(std::string_view property_name) {
    return Quoted(property_name);
}

std::expected<void, std::string> RequireEditableAndGivenItsValueKind(
    const PropertyDescription& description, const PropertyValue& value) {
    const std::string quoted_property_name = QuotePropertyName(description.name);
    if (!description.editable) {
        return std::unexpected(quoted_property_name + " is read-only");
    }
    const PropertyValueKind given_kind = ValueKindOf(value);
    if (given_kind != description.value_kind) {
        return std::unexpected(quoted_property_name + " holds " +
                               DescribeValueKind(description.value_kind) +
                               ", but the value given is " + DescribeValueKind(given_kind));
    }
    return {};
}

std::string RefuseTokenOutsideChoices(std::string_view property_name,
                                      std::span<const std::string_view> tokens,
                                      const std::string& given) {
    std::string choices;
    for (const std::string_view token : tokens) {
        choices += (choices.empty() ? "" : ", ") + Quoted(token);
    }
    return Quoted(property_name) + " is one of " + choices + ", not " + Quoted(given);
}

}  // namespace orvd::vehicle_project_editing::internal
