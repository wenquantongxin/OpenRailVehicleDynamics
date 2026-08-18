// Verifies that the toolchain this project is configured with compiled the
// OpenMP pragmas into real parallel semantics and linked a runtime that
// forms real teams.
//
// This is not a formality. A toolchain can accept an OpenMP-looking flag,
// link an OpenMP runtime, and still drop every pragma: Clang's
// -fopenmp=libgomp does exactly that, with no diagnostic even under
// -Wall -Wextra -Wpedantic. The resulting binary answers omp_get_max_threads
// and friends plausibly while thread zero does all the work, and because this
// project's physics is bit-identical across team sizes, every numerical test
// still passes. Only a check that observes the team itself can tell that
// build apart from a parallel one.
//
// The two checks below use the two pragma shapes the product actually ships:
// the contact batch's combined statically scheduled loop
// (wheel_rail_contact_force_plan.cc) and the Jacobian's parallel region with
// a dynamically scheduled column loop (system_continuous_state_advancer.cc).
#include <omp.h>

#ifndef _OPENMP
#error "OpenMP compile semantics are required: _OPENMP is not defined"
#endif

#include <array>
#include <cstdio>
#include <string_view>
#include <utility>

namespace {

// Two workers are the load-bearing minimum: they separate a real team from
// the silently serial build. The full worker counts are qualified by the
// dynamics runs themselves, not here.
constexpr int kRequestedWorkers = 2;
constexpr std::size_t kItems = 8;

[[nodiscard]] std::size_t DistinctCount(
    const std::array<int, kItems>& ordinals) {
    std::array<bool, kItems> seen{};
    for (const int ordinal : ordinals) {
        if (ordinal >= 0 && ordinal < static_cast<int>(kItems)) {
            seen[static_cast<std::size_t>(ordinal)] = true;
        }
    }
    std::size_t distinct = 0;
    for (const bool flag : seen) {
        distinct += flag ? 1U : 0U;
    }
    return distinct;
}

[[nodiscard]] const char* VerifyStaticBatchTeam() {
    if (omp_in_parallel() != 0) {
        return "already inside a parallel region before the check began";
    }
    std::array<int, kItems> ordinals{};
    ordinals.fill(-1);
    int parallel_flags = 0;
#pragma omp parallel for schedule(static) num_threads(kRequestedWorkers)
    for (int item = 0; item < static_cast<int>(kItems); ++item) {
        ordinals[static_cast<std::size_t>(item)] = omp_get_thread_num();
        if (item == 0 && omp_in_parallel() != 0) {
            parallel_flags = 1;
        }
    }
    if (parallel_flags == 0) {
        return "the statically scheduled batch parallel region was inactive "
               "or serialized";
    }
    if (DistinctCount(ordinals) < 2) {
        return "the statically scheduled batch ran on a single worker";
    }
    return nullptr;
}

[[nodiscard]] const char* VerifyDynamicColumnTeam() {
    std::array<int, kRequestedWorkers> parallel_flags{};
    std::array<int, kRequestedWorkers> worker_iteration_counts{};
#pragma omp parallel num_threads(kRequestedWorkers)
    {
        const int worker = omp_get_thread_num();
        if (worker >= 0 && worker < kRequestedWorkers) {
            parallel_flags[static_cast<std::size_t>(worker)] =
                omp_in_parallel() != 0 ? 1 : 0;
        }
#pragma omp for schedule(dynamic, 1)
        for (int column = 0; column < static_cast<int>(kItems); ++column) {
            static_cast<void>(column);
            ++worker_iteration_counts[static_cast<std::size_t>(worker)];
        }
    }
    if (parallel_flags[0] == 0 || parallel_flags[1] == 0) {
        return "the dynamic column parallel region was inactive or "
               "serialized";
    }
    int total_iterations = 0;
    for (const int worker_iteration_count : worker_iteration_counts) {
        total_iterations += worker_iteration_count;
    }
    if (total_iterations != static_cast<int>(kItems)) {
        return "the dynamically scheduled column worksharing loop did not "
               "execute each item exactly once";
    }
    return nullptr;
}

}  // namespace

int main() {
    // The product's contact batch and Jacobian both run with dynamic team
    // sizing disabled; the probe measures the same contract.
    omp_set_dynamic(0);
    for (const auto& [check_name, check] :
         {std::pair{std::string_view{"static contact-batch shape"},
                    &VerifyStaticBatchTeam},
          std::pair{std::string_view{"dynamic Jacobian-column shape"},
                    &VerifyDynamicColumnTeam}}) {
        if (const char* failure = check(); failure != nullptr) {
            std::fprintf(stderr, "%.*s FAILED: %s\n",
                         static_cast<int>(check_name.size()),
                         check_name.data(), failure);
            return 1;
        }
    }
    std::printf(
        "OpenMP parallel execution verified (_OPENMP=%d, max threads %d)\n",
        _OPENMP, omp_get_max_threads());
    return 0;
}
