#pragma once

/// @file
/// Source-tree-only identities for implemented system integration recipes.

#include <optional>
#include <stdexcept>
#include <string_view>

namespace orvd::integrators::internal {

// This closed set contains only recipes with a real backend and consumer.
// Each mechanical identity has its coordinate core, bridge and physical-state
// adapter. Configuration is carried by the corresponding concrete alternative.
enum class SystemContinuousStateIntegrationRecipe {
    kCvodeBdf2,
    kCvodeBdf5,
    kRadau5,
    kNewmark,
    kZhai,
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
        case SystemContinuousStateIntegrationRecipe::kNewmark:
            return "newmark";
        case SystemContinuousStateIntegrationRecipe::kZhai:
            return "zhai";
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
        case SystemContinuousStateIntegrationRecipe::kNewmark:
        case SystemContinuousStateIntegrationRecipe::kZhai:
            return std::nullopt;
    }
    throw std::invalid_argument(
        "system integration recipe: unsupported identity");
}

}  // namespace orvd::integrators::internal
