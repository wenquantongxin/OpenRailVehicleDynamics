#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "qualification_run_summary.h"
#include "qualification_sample_clock.h"

namespace orvd::dynamics_qualification {

// One internal IRW qualification configuration. G70/G71 leave the
// track-irregularity identity empty; G72 supplies the frozen AAR5 identity.
// The runner selects a closed required/forbidden recipe rather than exposing
// an arbitrary vehicle API. This type is not installed or exported.
struct IrwQualificationRunConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::optional<std::string> track_irregularity_identifier;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
    std::int64_t sample_period_nanoseconds{};
    std::optional<QualificationSampleRefinement> local_sample_refinement;
};

// One closed P179-personality control qualification. The AAR5 identity,
// 100 Hz event period and IRW topology are fixed by the recipe and loaded
// assets; this surface does not expose an arbitrary controller or event bus.
struct IrwP179ControlledQualificationRunConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::filesystem::path controller_configuration_path;
    std::filesystem::path torque_conditioner_configuration_path;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
};

struct IrwP179ControlledQualificationSummary final {
    std::size_t observation_count{};
    std::size_t control_audit_count{};
    std::size_t positive_hold_interval_count{};
    std::size_t backend_synchronization_count{};
    double advance_and_synchronization_wall_seconds{};
    double control_wall_seconds{};
    double observation_wall_seconds{};
    double data_and_metadata_write_wall_seconds{};
    double maximum_generalized_force_residual_inf_norm{};
    double maximum_absolute_virtual_power_residual_watts{};
    integrators::ContinuousStateIntegrationStatistics integration_statistics;
    Eigen::VectorXd terminal_continuous_state;
};

// Runs one passive IRW qualification in a private assembled system. A
// successful call publishes one complete directory atomically; failure leaves
// no destination and cannot mutate another qualification context.
[[nodiscard]] QualificationRunSummary RunIrwQualification(
    const IrwQualificationRunConfiguration& configuration);

// Runs the real H3/R300/frozen-AAR5 closed-loop IRW personality. Event zero is
// installed before backend construction; nonterminal events explicitly
// synchronize the backend; the committed terminal result is audit-only for
// future time because it owns no positive-duration hold interval.
[[nodiscard]] IrwP179ControlledQualificationSummary
RunIrwP179ControlledQualification(
    const IrwP179ControlledQualificationRunConfiguration& configuration);

}  // namespace orvd::dynamics_qualification
