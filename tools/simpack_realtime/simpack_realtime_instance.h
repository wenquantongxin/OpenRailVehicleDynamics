#pragma once

#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace orvd::simpack_realtime {

struct SimpackRealtimeInstanceConfiguration final {
    std::filesystem::path installation_path;
    std::filesystem::path model_path;
    std::filesystem::path log_path;
    std::string cpu_assignment;
    double communication_timeout_seconds{1.0};
    int verbose_level{0};
};

// One owning API-v2 direct-call session.
//
// Construction launches the solver, loads the model and captures its exact
// input/output ABI. Destruction always calls SpckRtFinish and therefore releases
// the solver process, message queues and licence owned by this instance.
class SimpackRealtimeInstance final {
   public:
    explicit SimpackRealtimeInstance(
        SimpackRealtimeInstanceConfiguration configuration);
    ~SimpackRealtimeInstance();

    SimpackRealtimeInstance(const SimpackRealtimeInstance&) = delete;
    SimpackRealtimeInstance& operator=(const SimpackRealtimeInstance&) = delete;
    SimpackRealtimeInstance(SimpackRealtimeInstance&&) = delete;
    SimpackRealtimeInstance& operator=(SimpackRealtimeInstance&&) = delete;

    [[nodiscard]] const std::filesystem::path& model_path() const noexcept;
    [[nodiscard]] const std::vector<std::string>& input_names() const noexcept;
    [[nodiscard]] const std::vector<std::string>& output_names() const noexcept;
    [[nodiscard]] const std::vector<double>& initial_input_values()
        const noexcept;

    [[nodiscard]] std::vector<double> ReadOutputs() const;
    void SetInitialInputs(std::span<const double> values);
    void Start();
    void SetInputs(std::span<const double> values);
    void Advance(double target_time_seconds);

   private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::simpack_realtime
