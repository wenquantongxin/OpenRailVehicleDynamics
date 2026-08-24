#pragma once

/// @file
/// Source-tree-only identities for implemented system integration recipes.

#include <optional>
#include <stdexcept>
#include <string_view>

namespace orvd::integrators::internal {

// This closed set contains only recipes with a real backend and consumer.
// Newmark or Zhai may be added only after their distinct mechanical-state
// contracts and implementations exist.
enum class SystemContinuousStateIntegrationRecipe {
    kCvodeBdf2,
    kCvodeBdf5,
    kRadau5,
};

[[nodiscard]] constexpr std::string_view IntegrationRecipeIdentifier(
    SystemContinuousStateIntegrationRecipe recipe) {
    switch (recipe) {
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf2:
            return "cvode_bdf2";
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf5:
            return "cvode_bdf5";
        case SystemContinuousStateIntegrationRecipe::kRadau5:
            return "radau5";
    }
    throw std::invalid_argument(
        "system integration recipe: unsupported identity");
}

[[nodiscard]] constexpr std::optional<int> MaximumBdfOrderForRecipe(
    SystemContinuousStateIntegrationRecipe recipe) {
    switch (recipe) {
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf2:
            return 2;
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf5:
            return 5;
        case SystemContinuousStateIntegrationRecipe::kRadau5:
            return std::nullopt;
    }
    throw std::invalid_argument(
        "system integration recipe: unsupported identity");
}

}  // namespace orvd::integrators::internal
