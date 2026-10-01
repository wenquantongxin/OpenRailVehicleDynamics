#pragma once

#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace orvd::dynamics_qualification {

// Application presets and result labels. All construction uses the library's
// public typed configuration; this enum is not a second backend factory.
enum class QualificationIntegrationMethod {
    kCvodeBdf2, kCvodeBdf5, kRadau5, kNewmark, kZhai,
};

[[nodiscard]] constexpr std::string_view IntegrationRecipeIdentifier(
    QualificationIntegrationMethod method) {
    switch (method) {
        case QualificationIntegrationMethod::kCvodeBdf2: return "cvode_bdf2";
        case QualificationIntegrationMethod::kCvodeBdf5: return "cvode_bdf5";
        case QualificationIntegrationMethod::kRadau5: return "radau5";
        case QualificationIntegrationMethod::kNewmark: return "newmark";
        case QualificationIntegrationMethod::kZhai: return "zhai";
    }
    throw std::invalid_argument("qualification: unsupported integration method");
}

[[nodiscard]] constexpr QualificationIntegrationMethod ParseIntegrationMethod(
    std::string_view identifier) {
    for (const auto method : {QualificationIntegrationMethod::kCvodeBdf2,
                             QualificationIntegrationMethod::kCvodeBdf5,
                             QualificationIntegrationMethod::kRadau5,
                             QualificationIntegrationMethod::kNewmark,
                             QualificationIntegrationMethod::kZhai}) {
        if (IntegrationRecipeIdentifier(method) == identifier) return method;
    }
    throw std::invalid_argument("qualification: unsupported integration method identifier");
}

[[nodiscard]] constexpr std::optional<int> MaximumBdfOrderForRecipe(
    QualificationIntegrationMethod method) {
    switch (method) {
        case QualificationIntegrationMethod::kCvodeBdf2: return 2;
        case QualificationIntegrationMethod::kCvodeBdf5: return 5;
        case QualificationIntegrationMethod::kRadau5:
        case QualificationIntegrationMethod::kNewmark:
        case QualificationIntegrationMethod::kZhai: return std::nullopt;
    }
    throw std::invalid_argument("qualification: unsupported integration method");
}

}  // namespace orvd::dynamics_qualification
