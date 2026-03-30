#ifndef GO2_X5_ARM_OUTPUT_GUARD_HPP
#define GO2_X5_ARM_OUTPUT_GUARD_HPP

#include <vector>

namespace Go2X5ArmOutputGuard
{

struct Context
{
    int num_dofs = 0;
    int arm_joint_start_index = 0;
    int arm_command_size = 0;
    bool arm_hold_enabled = false;
    bool arm_split_control_enabled = false;
    bool arm_bridge_require_state = false;
    bool arm_bridge_state_fresh = true;
    std::vector<float> arm_hold_position;
};

bool HasValidHoldRange(const Context& context);
bool ShouldApplyHoldOverride(const Context& context);
bool ShouldApplyStaleBridgeOverride(const Context& context);
bool ApplyHoldOverride(std::vector<float>* output_pos,
                       std::vector<float>* output_vel,
                       const Context& context);
bool ApplyOutputGuards(std::vector<float>* output_pos,
                       std::vector<float>* output_vel,
                       const Context& context);

} // namespace Go2X5ArmOutputGuard

#endif // GO2_X5_ARM_OUTPUT_GUARD_HPP
