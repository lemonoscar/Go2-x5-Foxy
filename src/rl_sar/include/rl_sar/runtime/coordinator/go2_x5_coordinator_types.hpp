#ifndef RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_TYPES_HPP
#define RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_TYPES_HPP

#include <array>
#include <cstdint>

#include "rl_sar/protocol/go2_x5_protocol.hpp"
#include "rl_sar/runtime/supervisor/go2_x5_supervisor_types.hpp"

namespace rl_sar::runtime::coordinator
{

struct Config
{
    uint64_t body_command_expire_ns = 15'000'000ULL;
    uint64_t arm_command_expire_ns = 15'000'000ULL;
    std::array<float, protocol::kBodyJointCount> default_leg_q{};
    std::array<float, protocol::kBodyJointCount> action_scale{};
    std::array<float, protocol::kBodyJointCount> rl_kp{};
    std::array<float, protocol::kBodyJointCount> rl_kd{};
    std::array<float, protocol::kBodyJointCount> torque_limits{};
    std::array<float, protocol::kBodyJointCount> safe_stand_q{};
};

struct Input
{
    uint64_t now_monotonic_ns = 0;
    Go2X5Supervisor::Mode mode = Go2X5Supervisor::Mode::Boot;

    bool has_body_state = false;
    protocol::BodyStateFrame body_state;

    bool has_arm_state = false;
    protocol::ArmStateFrame arm_state;

    bool has_operator_command = false;
    protocol::OperatorCommandFrame operator_command;

    bool has_policy_command = false;
    protocol::DogPolicyCommandFrame dog_policy_command;

    bool has_arm_command = false;
    protocol::ArmCommandFrame arm_command;

    bool arm_backend_valid = true;
    bool arm_tracking_error_high = false;
};

struct Output
{
    protocol::BodyCommandFrame body_command;
    protocol::ArmCommandFrame arm_command;
    bool body_command_valid = false;
    bool arm_command_valid = false;
    bool body_hold = false;
    bool arm_hold = false;
    bool policy_applied = false;
    bool arm_passthrough_applied = false;
};

}  // namespace rl_sar::runtime::coordinator

#endif  // RL_SAR_RUNTIME_COORDINATOR_GO2_X5_COORDINATOR_TYPES_HPP
