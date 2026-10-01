#pragma once

#include <stdexcept>
#include <string_view>
#include <utility>
#include "orvd/integrators/system_integration_configuration.h"

namespace orvd::integrators::test {
enum class TestIntegrationMethod { kCvodeBdf2, kCvodeBdf5, kRadau5, kNewmark, kZhai };
inline std::string_view MethodIdentifier(TestIntegrationMethod method) {
    switch (method) {
        case TestIntegrationMethod::kCvodeBdf2: return "cvode_bdf2";
        case TestIntegrationMethod::kCvodeBdf5: return "cvode_bdf5";
        case TestIntegrationMethod::kRadau5: return "radau5";
        case TestIntegrationMethod::kNewmark: return "newmark";
        case TestIntegrationMethod::kZhai: return "zhai";
    }
    throw std::invalid_argument("unsupported test method");
}
inline SystemIntegrationMethodConfiguration OdeMethod(
    TestIntegrationMethod method, ContinuousStateErrorTolerances tolerances) {
    switch (method) {
        case TestIntegrationMethod::kCvodeBdf2: return CvodeBdf2Configuration{std::move(tolerances)};
        case TestIntegrationMethod::kCvodeBdf5: return CvodeBdf5Configuration{std::move(tolerances)};
        case TestIntegrationMethod::kRadau5: return Radau5Configuration{std::move(tolerances)};
        default: throw std::invalid_argument("test requires an ODE method");
    }
}
inline NewmarkConfiguration NewmarkSettings(double h, double scale = 1e-11) {
    return {h, {12, {scale, scale, scale, 1.0},
                    {scale, scale, scale, 1.0},
                    {scale, scale, scale, 1.0},
                    {scale, scale, 1.0}}};
}
}  // namespace orvd::integrators::test
