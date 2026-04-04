#include "rl_sar/runtime/coordinator/go2_x5_coordinator.hpp"

#include <algorithm>
#include <cmath>

namespace rl_sar::runtime::coordinator
{
namespace
{

template <size_t N>
float ClampAbs(float value, const std::array<float, N>& limits, size_t idx)
{
    const float limit = (idx < limits.size()) ? std::max(0.0f, limits[idx]) : 0.0f;
    if (limit <= 0.0f)
    {
        return value;
    }
    return std::max(-limit, std::min(limit, value));
}

template <size_t N>
void FillBodyHeader(protocol::BodyCommandFrame* frame,
                    Go2X5Supervisor::Mode mode,
                    uint64_t now_ns,
                    uint64_t expire_ns)
{
    frame->header.msg_type = protocol::FrameType::BodyCommand;
    frame->header.mode = static_cast<uint16_t>(mode);
    frame->header.source_monotonic_ns = now_ns;
    frame->header.publish_monotonic_ns = now_ns;
    frame->header.validity_flags = protocol::kValidityPayloadValid;
    frame->joint_count = protocol::kBodyJointCount;
    frame->command_expire_ns = expire_ns;
    (void)N;
}

void FillArmHeader(protocol::ArmCommandFrame* frame,
                   Go2X5Supervisor::Mode mode,
                   uint64_t now_ns,
                   uint64_t expire_ns)
{
    frame->header.msg_type = protocol::FrameType::ArmCommand;
    frame->header.mode = static_cast<uint16_t>(mode);
    frame->header.source_monotonic_ns = now_ns;
    frame->header.publish_monotonic_ns = now_ns;
    frame->header.validity_flags = protocol::kValidityPayloadValid;
    frame->joint_count = protocol::kArmJointCount;
    frame->command_expire_ns = expire_ns;
}

protocol::BodyCommandFrame MakeSafeStandBodyCommand(const Config& config,
                                                    const Input& input)
{
    protocol::BodyCommandFrame frame;
    FillBodyHeader<protocol::kBodyJointCount>(
        &frame, input.mode, input.now_monotonic_ns, input.now_monotonic_ns + config.body_command_expire_ns);
    frame.header.validity_flags |= protocol::kValidityFallbackGenerated;
    for (size_t i = 0; i < protocol::kBodyJointCount; ++i)
    {
        frame.q[i] = config.safe_stand_q[i];
        frame.dq[i] = 0.0f;
        frame.kp[i] = config.rl_kp[i];
        frame.kd[i] = config.rl_kd[i];
        if (input.has_body_state)
        {
            const float tau = config.rl_kp[i] * (frame.q[i] - input.body_state.leg_q[i]) -
                              config.rl_kd[i] * input.body_state.leg_dq[i];
            frame.tau[i] = ClampAbs(tau, config.torque_limits, i);
        }
        else
        {
            frame.tau[i] = 0.0f;
        }
    }
    return frame;
}

protocol::ArmCommandFrame MakeHoldArmCommand(const Config& config, const Input& input)
{
    protocol::ArmCommandFrame frame;
    FillArmHeader(&frame, input.mode, input.now_monotonic_ns, input.now_monotonic_ns + config.arm_command_expire_ns);
    frame.header.validity_flags |= protocol::kValidityFallbackGenerated;
    if (input.has_arm_state)
    {
        frame.q = input.arm_state.q;
    }
    else if (input.has_arm_command)
    {
        frame.q = input.arm_command.q;
    }
    for (size_t i = 0; i < protocol::kArmJointCount; ++i)
    {
        frame.dq[i] = 0.0f;
        frame.kp[i] = 0.0f;
        frame.kd[i] = 0.0f;
        frame.tau[i] = 0.0f;
    }
    return frame;
}

bool CanUseArmCommand(const Input& input)
{
    if (!input.has_arm_command)
    {
        return false;
    }
    if (protocol::IsCommandExpired(input.now_monotonic_ns, input.arm_command.command_expire_ns))
    {
        return false;
    }
    return input.arm_command.joint_count == protocol::kArmJointCount;
}

}  // namespace

HybridMotionCoordinator::HybridMotionCoordinator(Config config)
    : config_(std::move(config))
{
}

Output HybridMotionCoordinator::Step(const Input& input) const
{
    Output output;

    switch (input.mode)
    {
    case Go2X5Supervisor::Mode::Ready:
    case Go2X5Supervisor::Mode::ManualArm:
        if (CanUseArmCommand(input))
        {
            output.arm_command = input.arm_command;
            output.arm_command.header.mode = static_cast<uint16_t>(input.mode);
            output.arm_command_valid = true;
            output.arm_passthrough_applied = true;
        }
        break;

    case Go2X5Supervisor::Mode::RlDogOnlyActive:
        if (input.has_policy_command &&
            input.dog_policy_command.action_dim == protocol::kDogJointCount)
        {
            FillBodyHeader<protocol::kBodyJointCount>(
                &output.body_command,
                input.mode,
                input.now_monotonic_ns,
                input.now_monotonic_ns + config_.body_command_expire_ns);
            output.body_command.header.validity_flags |= protocol::kValidityFallbackGenerated;
            for (size_t i = 0; i < protocol::kBodyJointCount; ++i)
            {
                const float target_q =
                    config_.default_leg_q[i] + input.dog_policy_command.leg_action[i] * config_.action_scale[i];
                output.body_command.q[i] = target_q;
                output.body_command.dq[i] = 0.0f;
                output.body_command.kp[i] = config_.rl_kp[i];
                output.body_command.kd[i] = config_.rl_kd[i];
                if (input.has_body_state)
                {
                    const float tau = config_.rl_kp[i] * (target_q - input.body_state.leg_q[i]) -
                                      config_.rl_kd[i] * input.body_state.leg_dq[i];
                    output.body_command.tau[i] = ClampAbs(tau, config_.torque_limits, i);
                }
                else
                {
                    output.body_command.tau[i] = 0.0f;
                }
            }
            output.body_command_valid = true;
            output.policy_applied = true;
        }

        if (CanUseArmCommand(input) && input.arm_backend_valid && !input.arm_tracking_error_high)
        {
            output.arm_command = input.arm_command;
            output.arm_command.header.mode = static_cast<uint16_t>(input.mode);
            output.arm_command_valid = true;
            output.arm_passthrough_applied = true;
        }
        break;

    case Go2X5Supervisor::Mode::DegradedArm:
        output.body_command = MakeSafeStandBodyCommand(config_, input);
        output.body_command_valid = true;
        output.body_hold = true;
        output.arm_command = MakeHoldArmCommand(config_, input);
        output.arm_command_valid = true;
        output.arm_hold = true;
        break;

    case Go2X5Supervisor::Mode::DegradedBody:
        output.body_command = MakeSafeStandBodyCommand(config_, input);
        output.body_command_valid = true;
        output.body_hold = true;
        break;

    case Go2X5Supervisor::Mode::Boot:
    case Go2X5Supervisor::Mode::Probe:
    case Go2X5Supervisor::Mode::Passive:
    case Go2X5Supervisor::Mode::SoftStop:
    case Go2X5Supervisor::Mode::FaultLatched:
        break;
    }

    return output;
}

}  // namespace rl_sar::runtime::coordinator
