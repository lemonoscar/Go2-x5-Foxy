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

uint64_t ResolvePolicyReceiveTimeNs(const protocol::DogPolicyCommandFrame& frame, uint64_t now_ns)
{
    const uint64_t source_ns =
        frame.header.publish_monotonic_ns != 0
            ? frame.header.publish_monotonic_ns
            : frame.header.source_monotonic_ns;
    if (source_ns == 0 || source_ns > now_ns)
    {
        return now_ns;
    }
    return source_ns;
}

std::array<float, protocol::kBodyJointCount> DecodePolicyTargetQ(
    const Config& config,
    const protocol::DogPolicyCommandFrame& policy_command)
{
    std::array<float, protocol::kBodyJointCount> target_q{};
    for (size_t i = 0; i < protocol::kBodyJointCount; ++i)
    {
        target_q[i] = config.default_leg_q[i] + policy_command.leg_action[i] * config.action_scale[i];
    }
    return target_q;
}

protocol::BodyCommandFrame MakeTrackingBodyCommand(
    const Config& config,
    const Input& input,
    const std::array<float, protocol::kBodyJointCount>& target_q,
    const std::array<float, protocol::kBodyJointCount>& target_dq,
    bool fallback_generated)
{
    protocol::BodyCommandFrame frame;
    FillBodyHeader<protocol::kBodyJointCount>(
        &frame, input.mode, input.now_monotonic_ns, input.now_monotonic_ns + config.body_command_expire_ns);
    if (fallback_generated)
    {
        frame.header.validity_flags |= protocol::kValidityFallbackGenerated;
    }
    for (size_t i = 0; i < protocol::kBodyJointCount; ++i)
    {
        frame.q[i] = target_q[i];
        frame.dq[i] = target_dq[i];
        frame.kp[i] = config.rl_kp[i];
        frame.kd[i] = config.rl_kd[i];
        if (input.has_body_state)
        {
            const float tau = config.rl_kp[i] * (frame.q[i] - input.body_state.leg_q[i]) +
                              config.rl_kd[i] * (frame.dq[i] - input.body_state.leg_dq[i]);
            frame.tau[i] = ClampAbs(tau, config.torque_limits, i);
        }
        else
        {
            frame.tau[i] = 0.0f;
        }
    }
    return frame;
}

protocol::BodyCommandFrame MakeSafeStandBodyCommand(const Config& config,
                                                    const Input& input)
{
    std::array<float, protocol::kBodyJointCount> target_dq{};
    return MakeTrackingBodyCommand(config, input, config.safe_stand_q, target_dq, true);
}

protocol::BodyCommandFrame MakeSmoothedSafeStandBodyCommand(
    const Config& config,
    const Input& input,
    const std::array<float, protocol::kBodyJointCount>& target_q,
    const std::array<float, protocol::kBodyJointCount>& target_dq)
{
    return MakeTrackingBodyCommand(config, input, target_q, target_dq, true);
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
    , fallback_smoother_(config_.fallback_smoother)
    , rl_handover_smoother_(config_.fallback_smoother)
{
}

void HybridMotionCoordinator::UpdatePolicyState(const Input& input) const
{
    policy_state_.current_cmd_from_fresh_sample = false;
    if (!input.has_policy_command || input.dog_policy_command.action_dim != protocol::kDogJointCount)
    {
        return;
    }

    const uint64_t seq = input.dog_policy_command.header.seq;
    if (!policy_state_.valid || seq != policy_state_.seq)
    {
        policy_state_.cmd = input.dog_policy_command;
        policy_state_.receive_monotonic_ns =
            ResolvePolicyReceiveTimeNs(input.dog_policy_command, input.now_monotonic_ns);
        policy_state_.seq = seq;
        policy_state_.valid = true;
        policy_state_.current_cmd_from_fresh_sample = true;
    }
}

std::pair<uint64_t, bool> HybridMotionCoordinator::EvaluatePolicyFreshness(
    uint64_t now_monotonic_ns) const
{
    if (!policy_state_.valid)
    {
        return {0ULL, false};
    }

    const uint64_t receive_ns =
        policy_state_.receive_monotonic_ns > now_monotonic_ns ? now_monotonic_ns : policy_state_.receive_monotonic_ns;
    const uint64_t age_ns = now_monotonic_ns - receive_ns;
    return {age_ns, age_ns <= config_.policy_fresh_threshold_ns};
}

Output HybridMotionCoordinator::Step(const Input& input) const
{
    Output output;

    UpdatePolicyState(input);
    const auto [policy_age_ns, policy_is_fresh] = EvaluatePolicyFreshness(input.now_monotonic_ns);

    const bool degraded_mode =
        input.mode == Go2X5Supervisor::Mode::DegradedArm ||
        input.mode == Go2X5Supervisor::Mode::DegradedBody;
    const bool entering_new_mode = input.mode != last_mode_;
    if (!degraded_mode)
    {
        fallback_smoother_.Reset();
        fallback_plan_active_ = false;
    }
    if (input.mode != Go2X5Supervisor::Mode::RlDogOnlyActive)
    {
        rl_handover_smoother_.Reset();
        rl_handover_plan_active_ = false;
    }
    else if (entering_new_mode)
    {
        rl_handover_smoother_.Reset();
        rl_handover_plan_active_ = false;
    }
    last_mode_ = input.mode;

    switch (input.mode)
    {
    case Go2X5Supervisor::Mode::Ready:
        if (CanUseArmCommand(input))
        {
            output.arm_command = input.arm_command;
            output.arm_command.header.mode = static_cast<uint16_t>(input.mode);
            output.arm_command_valid = true;
            output.arm_passthrough_applied = true;
        }
        break;

    case Go2X5Supervisor::Mode::ManualArm:
        output.body_command = MakeSafeStandBodyCommand(config_, input);
        output.body_command_valid = true;
        output.body_hold = true;
        if (CanUseArmCommand(input))
        {
            output.arm_command = input.arm_command;
            output.arm_command.header.mode = static_cast<uint16_t>(input.mode);
            output.arm_command_valid = true;
            output.arm_passthrough_applied = true;
        }
        break;

    case Go2X5Supervisor::Mode::RlDogOnlyActive:
        if (policy_state_.valid && policy_state_.cmd.action_dim == protocol::kDogJointCount)
        {
            const auto decoded_target_q = DecodePolicyTargetQ(config_, policy_state_.cmd);
            std::array<float, protocol::kBodyJointCount> target_q = decoded_target_q;
            std::array<float, protocol::kBodyJointCount> target_dq{};

            if (!rl_handover_plan_active_)
            {
                const auto current_q = input.has_body_state ? input.body_state.leg_q : config_.default_leg_q;
                const float handover_duration_sec =
                    static_cast<float>(config_.rl_handover_duration_ns) / 1e9f;
                rl_handover_smoother_.Plan(
                    current_q, decoded_target_q, input.now_monotonic_ns, handover_duration_sec);
                rl_handover_plan_active_ = true;
            }

            if (rl_handover_smoother_.IsActive(input.now_monotonic_ns))
            {
                target_q = rl_handover_smoother_.GetTargetPosition(input.now_monotonic_ns);
                target_dq = rl_handover_smoother_.GetTargetVelocity(input.now_monotonic_ns);
            }
            else
            {
                rl_handover_plan_active_ = false;
            }
            output.body_command =
                MakeTrackingBodyCommand(config_, input, target_q, target_dq, false);
            output.body_command_valid = true;
            output.policy_applied = true;
            output.policy_is_fresh = policy_is_fresh;
            output.current_cmd_from_fresh_sample = policy_state_.current_cmd_from_fresh_sample;
            output.policy_age_ns = policy_age_ns;
            output.policy_seq = policy_state_.seq;
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
    case Go2X5Supervisor::Mode::DegradedBody:
        if (entering_new_mode)
        {
            fallback_plan_active_ = false;
        }
        if (!fallback_plan_active_)
        {
            const std::array<float, protocol::kBodyJointCount> current_q =
                input.has_body_state ? input.body_state.leg_q : config_.safe_stand_q;
            fallback_smoother_.Plan(current_q, config_.safe_stand_q, input.now_monotonic_ns);
            fallback_plan_active_ = true;
        }
        output.body_command = MakeSmoothedSafeStandBodyCommand(
            config_,
            input,
            fallback_smoother_.GetTargetPosition(input.now_monotonic_ns),
            fallback_smoother_.GetTargetVelocity(input.now_monotonic_ns));
        output.body_command_valid = true;
        output.body_hold = true;
        if (input.mode == Go2X5Supervisor::Mode::DegradedArm)
        {
            output.arm_command = MakeHoldArmCommand(config_, input);
            output.arm_command_valid = true;
            output.arm_hold = true;
        }
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
