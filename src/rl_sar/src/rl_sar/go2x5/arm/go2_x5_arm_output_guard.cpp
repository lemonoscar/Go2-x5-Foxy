#include "rl_sar/go2x5/arm_output_guard.hpp"

#include <algorithm>

namespace Go2X5ArmOutputGuard
{

namespace
{

bool HasEnoughHoldPose(const Context& context)
{
    return context.arm_command_size > 0 &&
           context.arm_hold_position.size() >= static_cast<size_t>(context.arm_command_size);
}

} // namespace

bool HasValidHoldRange(const Context& context)
{
    if (!HasEnoughHoldPose(context))
    {
        return false;
    }

    const int arm_start = std::max(0, context.arm_joint_start_index);
    return context.num_dofs > 0 && (arm_start + context.arm_command_size) <= context.num_dofs;
}

bool ShouldApplyHoldOverride(const Context& context)
{
    return context.arm_hold_enabled && HasValidHoldRange(context);
}

bool ShouldApplyStaleBridgeOverride(const Context& context)
{
    return context.arm_split_control_enabled &&
           context.arm_bridge_require_state &&
           !context.arm_bridge_state_fresh &&
           HasValidHoldRange(context);
}

bool ApplyHoldOverride(std::vector<float>* output_pos,
                       std::vector<float>* output_vel,
                       const Context& context)
{
    if (output_pos == nullptr || output_vel == nullptr || !HasValidHoldRange(context))
    {
        return false;
    }

    const int arm_start = std::max(0, context.arm_joint_start_index);
    for (int i = 0; i < context.arm_command_size; ++i)
    {
        const size_t idx = static_cast<size_t>(arm_start + i);
        if (idx < output_pos->size())
        {
            (*output_pos)[idx] = context.arm_hold_position[static_cast<size_t>(i)];
        }
        if (idx < output_vel->size())
        {
            (*output_vel)[idx] = 0.0f;
        }
    }

    return true;
}

bool ApplyOutputGuards(std::vector<float>* output_pos,
                       std::vector<float>* output_vel,
                       const Context& context)
{
    bool applied = false;
    if (ShouldApplyHoldOverride(context))
    {
        applied = ApplyHoldOverride(output_pos, output_vel, context);
    }
    if (ShouldApplyStaleBridgeOverride(context))
    {
        applied = ApplyHoldOverride(output_pos, output_vel, context) || applied;
    }
    return applied;
}

} // namespace Go2X5ArmOutputGuard
