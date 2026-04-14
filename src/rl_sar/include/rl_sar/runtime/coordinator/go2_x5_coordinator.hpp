#ifndef RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_HPP
#define RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_HPP

#include <utility>

#include "rl_sar/runtime/coordinator/go2_x5_coordinator_types.hpp"

namespace rl_sar::runtime::coordinator
{

class HybridMotionCoordinator
{
public:
    explicit HybridMotionCoordinator(Config config = {});

    const Config& config() const { return config_; }

    Output Step(const Input& input) const;

private:
    struct PolicyState
    {
        protocol::DogPolicyCommandFrame cmd{};
        uint64_t receive_monotonic_ns = 0;
        uint64_t seq = 0;
        bool valid = false;
        bool current_cmd_from_fresh_sample = false;
    };

    void UpdatePolicyState(const Input& input) const;
    std::pair<uint64_t, bool> EvaluatePolicyFreshness(uint64_t now_monotonic_ns) const;

    Config config_;
    mutable PolicyState policy_state_{};
    mutable trajectory::FallbackSmoother fallback_smoother_;
    mutable Go2X5Supervisor::Mode last_mode_ = Go2X5Supervisor::Mode::Boot;
    mutable bool fallback_plan_active_ = false;
};

}  // namespace rl_sar::runtime::coordinator

#endif  // RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_HPP
