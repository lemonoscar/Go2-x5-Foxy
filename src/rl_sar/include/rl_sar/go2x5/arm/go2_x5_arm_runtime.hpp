#ifndef GO2_X5_ARM_RUNTIME_HPP
#define GO2_X5_ARM_RUNTIME_HPP

#include <vector>

#include "rl_sar/go2x5/control/go2_x5_control_logic.hpp"

namespace Go2X5ArmRuntime
{

struct CommandState
{
    int arm_command_size = 0;
    bool arm_hold_enabled = true;
    bool arm_topic_command_received = false;
    bool arm_command_initialized = false;
    int arm_command_smoothing_ticks = 0;
    int arm_command_smoothing_counter = 0;
    std::vector<float> arm_joint_command_latest;
    std::vector<float> arm_topic_command_latest;
    std::vector<float> arm_hold_position;
    std::vector<float> arm_command_smoothing_start;
    std::vector<float> arm_command_smoothing_target;
    std::vector<float> arm_command_smoothed;
};

struct InitializationConfig
{
    int arm_command_size = 0;
    int arm_joint_start_index = 0;
    bool arm_hold_enabled = true;
    float step_dt = 0.0f;
    float smoothing_time = 0.2f;
    std::vector<float> arm_hold_pose;
    std::vector<float> default_dof_pos;
};

CommandState BuildInitialCommandState(const InitializationConfig& config);
Go2X5ControlLogic::ArmRuntimeStateSnapshot CaptureSnapshot(const CommandState& state);
void RestoreSnapshotIfCompatible(CommandState* state,
                                 const Go2X5ControlLogic::ArmRuntimeStateSnapshot& snapshot);
bool CommandDifferent(const std::vector<float>& a, const std::vector<float>& b);
void ApplyHoldTarget(CommandState* state, const std::vector<float>& target);
std::vector<float> StepSmoothedCommand(CommandState* state, const std::vector<float>& desired_command);

} // namespace Go2X5ArmRuntime

#endif // GO2_X5_ARM_RUNTIME_HPP
