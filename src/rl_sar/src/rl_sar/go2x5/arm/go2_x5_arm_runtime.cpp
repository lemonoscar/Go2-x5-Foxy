#include "rl_sar/go2x5/arm_runtime.hpp"

#include <algorithm>
#include <cmath>

namespace Go2X5ArmRuntime
{
namespace
{

std::vector<float> ResolveInitialHoldPose(const InitializationConfig& config)
{
    if (config.arm_command_size <= 0)
    {
        return {};
    }

    std::vector<float> hold_position(static_cast<size_t>(config.arm_command_size), 0.0f);
    if (config.arm_hold_pose.size() == static_cast<size_t>(config.arm_command_size))
    {
        return config.arm_hold_pose;
    }

    const int arm_start = std::max(0, config.arm_joint_start_index);
    if (config.default_dof_pos.size() >= static_cast<size_t>(arm_start + config.arm_command_size))
    {
        hold_position.assign(
            config.default_dof_pos.begin() + static_cast<long>(arm_start),
            config.default_dof_pos.begin() + static_cast<long>(arm_start + config.arm_command_size));
        return hold_position;
    }

    if (config.default_dof_pos.size() >= static_cast<size_t>(config.arm_command_size))
    {
        const size_t fallback_arm_start =
            config.default_dof_pos.size() - static_cast<size_t>(config.arm_command_size);
        hold_position.assign(config.default_dof_pos.begin() + static_cast<long>(fallback_arm_start),
                             config.default_dof_pos.end());
    }
    return hold_position;
}

} // namespace

CommandState BuildInitialCommandState(const InitializationConfig& config)
{
    CommandState state;
    state.arm_command_size = config.arm_command_size;
    state.arm_hold_enabled = config.arm_hold_enabled;

    if (config.step_dt > 0.0f)
    {
        state.arm_command_smoothing_ticks =
            std::max(0, static_cast<int>(std::lround(config.smoothing_time / config.step_dt)));
    }

    if (state.arm_command_size <= 0)
    {
        return state;
    }

    state.arm_hold_position = ResolveInitialHoldPose(config);
    state.arm_joint_command_latest = state.arm_hold_position;
    state.arm_topic_command_latest = state.arm_hold_position;
    state.arm_topic_command_received = false;
    state.arm_command_smoothing_start = state.arm_hold_position;
    state.arm_command_smoothing_target = state.arm_hold_position;
    state.arm_command_smoothed = state.arm_hold_position;
    state.arm_command_initialized = true;
    return state;
}

Go2X5ControlLogic::ArmRuntimeStateSnapshot CaptureSnapshot(const CommandState& state)
{
    Go2X5ControlLogic::ArmRuntimeStateSnapshot snapshot;
    snapshot.arm_size = state.arm_command_size;
    snapshot.command_smoothing_counter = state.arm_command_smoothing_counter;
    snapshot.hold_enabled = state.arm_hold_enabled;
    snapshot.topic_command_received = state.arm_topic_command_received;
    snapshot.command_initialized = state.arm_command_initialized;
    snapshot.hold_position = state.arm_hold_position;
    snapshot.joint_command_latest = state.arm_joint_command_latest;
    snapshot.topic_command_latest = state.arm_topic_command_latest;
    snapshot.command_smoothing_start = state.arm_command_smoothing_start;
    snapshot.command_smoothing_target = state.arm_command_smoothing_target;
    snapshot.command_smoothed = state.arm_command_smoothed;
    return snapshot;
}

void RestoreSnapshotIfCompatible(CommandState* state,
                                 const Go2X5ControlLogic::ArmRuntimeStateSnapshot& snapshot)
{
    if (!state)
    {
        return;
    }

    auto current = CaptureSnapshot(*state);
    Go2X5ControlLogic::RestoreArmRuntimeStateIfCompatible(current, snapshot);

    state->arm_hold_enabled = current.hold_enabled;
    state->arm_topic_command_received = current.topic_command_received;
    state->arm_command_initialized = current.command_initialized;
    state->arm_command_smoothing_counter = current.command_smoothing_counter;
    state->arm_hold_position = std::move(current.hold_position);
    state->arm_joint_command_latest = std::move(current.joint_command_latest);
    state->arm_topic_command_latest = std::move(current.topic_command_latest);
    state->arm_command_smoothing_start = std::move(current.command_smoothing_start);
    state->arm_command_smoothing_target = std::move(current.command_smoothing_target);
    state->arm_command_smoothed = std::move(current.command_smoothed);
}

bool CommandDifferent(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.size() != b.size())
    {
        return true;
    }
    for (size_t i = 0; i < a.size(); ++i)
    {
        if (std::fabs(a[i] - b[i]) > 1e-5f)
        {
            return true;
        }
    }
    return false;
}

void ApplyHoldTarget(CommandState* state, const std::vector<float>& target)
{
    if (!state || state->arm_command_size <= 0 ||
        target.size() != static_cast<size_t>(state->arm_command_size))
    {
        return;
    }

    state->arm_hold_position = target;
    state->arm_joint_command_latest = target;
    state->arm_hold_enabled = true;
    if (!state->arm_command_initialized || state->arm_command_smoothed.size() != target.size())
    {
        state->arm_command_smoothing_start = target;
        state->arm_command_smoothing_target = target;
        state->arm_command_smoothed = target;
        state->arm_command_initialized = true;
        state->arm_command_smoothing_counter = state->arm_command_smoothing_ticks;
        return;
    }

    state->arm_command_smoothing_start = state->arm_command_smoothed;
    state->arm_command_smoothing_target = target;
    state->arm_command_smoothing_counter = 0;
}

std::vector<float> StepSmoothedCommand(CommandState* state, const std::vector<float>& desired_command)
{
    if (!state || desired_command.empty() ||
        desired_command.size() != static_cast<size_t>(state->arm_command_size))
    {
        return {};
    }

    if (!state->arm_command_initialized || state->arm_command_smoothed.size() != desired_command.size())
    {
        state->arm_command_smoothed = desired_command;
        state->arm_command_smoothing_start = desired_command;
        state->arm_command_smoothing_target = desired_command;
        state->arm_command_smoothing_counter = state->arm_command_smoothing_ticks;
        state->arm_command_initialized = true;
    }

    if (CommandDifferent(desired_command, state->arm_command_smoothing_target))
    {
        state->arm_command_smoothing_start = state->arm_command_smoothed;
        state->arm_command_smoothing_target = desired_command;
        state->arm_command_smoothing_counter = 0;
    }

    if (state->arm_command_smoothing_ticks <= 1)
    {
        state->arm_command_smoothed = state->arm_command_smoothing_target;
        return state->arm_command_smoothed;
    }

    const int ticks = state->arm_command_smoothing_ticks;
    const int c = std::min(state->arm_command_smoothing_counter + 1, ticks);
    const float alpha = static_cast<float>(c) / static_cast<float>(ticks);
    for (int i = 0; i < state->arm_command_size; ++i)
    {
        const float start = state->arm_command_smoothing_start[static_cast<size_t>(i)];
        const float target = state->arm_command_smoothing_target[static_cast<size_t>(i)];
        state->arm_command_smoothed[static_cast<size_t>(i)] = (1.0f - alpha) * start + alpha * target;
    }
    if (state->arm_command_smoothing_counter < ticks)
    {
        state->arm_command_smoothing_counter += 1;
    }
    return state->arm_command_smoothed;
}

} // namespace Go2X5ArmRuntime
