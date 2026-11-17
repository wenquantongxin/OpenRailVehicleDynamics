#pragma once

#include <expected>
#include <span>
#include <string>
#include <string_view>

#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

// What every property edit requires before it writes, whether the property is
// an element's or settings': the property can be edited, the value holds the
// property's value kind, and a token is one of its choices; and how a refusal
// names a property.

namespace orvd::vehicle_project_editing::internal {

// How a refusal names a property: 'mass_kilograms'.
[[nodiscard]] std::string QuotePropertyName(std::string_view property_name);

// Refuses a value for the property `description` describes when the property
// is read-only, or when the value holds another value kind than the
// property's.
[[nodiscard]] std::expected<void, std::string> RequireEditableAndGivenItsValueKind(
    const PropertyDescription& description, const PropertyValue& value);

// Why `given` is refused for the token property named `property_name`, whose
// choices are `tokens`.
[[nodiscard]] std::string RefuseTokenOutsideChoices(std::string_view property_name,
                                                    std::span<const std::string_view> tokens,
                                                    const std::string& given);

}  // namespace orvd::vehicle_project_editing::internal
