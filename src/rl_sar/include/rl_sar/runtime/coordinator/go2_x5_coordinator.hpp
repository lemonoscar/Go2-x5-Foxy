#ifndef RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_HPP
#define RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_HPP

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
    Config config_;
};

}  // namespace rl_sar::runtime::coordinator

#endif  // RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_HPP
