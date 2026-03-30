#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string ReadAll(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open: " << path << std::endl;
        std::abort();
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

void RequireContains(const std::string& content, const std::string& needle, const std::string& file)
{
    if (content.find(needle) == std::string::npos)
    {
        std::cerr << "Missing expected snippet in " << file << ": " << needle << std::endl;
        std::abort();
    }
}

void RequireNotContains(const std::string& content, const std::string& needle, const std::string& file)
{
    if (content.find(needle) != std::string::npos)
    {
        std::cerr << "Unexpected snippet in " << file << ": " << needle << std::endl;
        std::abort();
    }
}

}  // namespace

int main()
{
    const std::string source_dir = RL_SAR_SOURCE_DIR;
    const std::string real_file = source_dir + "/src/rl_sar/core/rl_real_go2_x5.cpp";
    const std::string state_io_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_state_io.cpp";
    const std::string state_management_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_state_management.cpp";
    const std::string utility_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_utility.cpp";
    const std::string safe_shutdown_file = source_dir + "/src/rl_sar/go2x5/safety/go2_x5_safe_shutdown.cpp";
    const std::string ros_file = source_dir + "/src/rl_sar/go2x5/comm/go2_x5_ros.cpp";
    const std::string sdk_file = source_dir + "/library/core/rl_sdk/rl_sdk.cpp";
    const std::string operator_file = source_dir + "/src/rl_sar/go2x5/control/go2_x5_operator_control.cpp";
    const std::string runtime_file = source_dir + "/src/rl_sar/go2x5/arm/go2_x5_arm_runtime.cpp";
    const std::string bridge_runtime_file = source_dir + "/src/rl_sar/go2x5/arm/go2_x5_arm_bridge_runtime.cpp";
    const std::string output_guard_file = source_dir + "/src/rl_sar/go2x5/arm/go2_x5_arm_output_guard.cpp";
    const std::string bridge_file = source_dir + "/scripts/arx_x5_bridge.py";
    const std::string base_file = source_dir + "/../../policy/go2_x5/base.yaml";
    const std::string config_file = source_dir + "/../../policy/go2_x5/robot_lab/config.yaml";

    const std::string real_content = ReadAll(real_file);
    const std::string state_io_content = ReadAll(state_io_file);
    const std::string state_management_content = ReadAll(state_management_file);
    const std::string utility_content = ReadAll(utility_file);
    const std::string safe_shutdown_content = ReadAll(safe_shutdown_file);
    const std::string ros_content = ReadAll(ros_file);
    const std::string sdk_content = ReadAll(sdk_file);
    const std::string operator_content = ReadAll(operator_file);
    const std::string runtime_content = ReadAll(runtime_file);
    const std::string bridge_runtime_content = ReadAll(bridge_runtime_file);
    const std::string output_guard_content = ReadAll(output_guard_file);
    const std::string bridge_content = ReadAll(bridge_file);
    const std::string base_content = ReadAll(base_file);
    const std::string config_content = ReadAll(config_file);

    RequireContains(real_content, "ClipWholeBodyCommand", real_file);
    RequireContains(state_io_content, "Arm passthrough blocked outside RL locomotion", state_io_file);
    RequireContains(safe_shutdown_content, "arm_safe_shutdown_active.store(true)", safe_shutdown_file);
    RequireContains(real_content, "Policy action dimension mismatch", real_file);
    RequireContains(utility_content, "Policy inference frequency:", utility_file);
    RequireContains(real_content, "HandleLoopException", real_file);
    RequireContains(ros_content, "Ignore /cmd_vel in exclusive real deploy control mode", ros_file);
    RequireContains(state_management_content, "arm_bridge_state_from_backend", state_management_file);
    RequireContains(state_io_content, "shadow-only state", state_io_file);
    RequireContains(state_io_content, "Arm bridge state is missing or stale. Ignoring shadow arm feedback until bridge recovers.", state_io_file);
    RequireContains(state_io_content, "Ignoring shadow arm feedback until a real backend state arrives.", state_io_file);
    RequireContains(real_content, "CaptureArmRuntimeStateLocked", real_file);
    RequireContains(runtime_content, "BuildInitialCommandState", runtime_file);
    RequireContains(runtime_content, "StepSmoothedCommand", runtime_file);
    RequireContains(bridge_runtime_content, "ReconcileConfiguration", bridge_runtime_file);
    RequireContains(bridge_runtime_content, "EvaluateReadDecision", bridge_runtime_file);
    RequireContains(output_guard_content, "ApplyOutputGuards", output_guard_file);
    RequireContains(output_guard_content, "ShouldApplyStaleBridgeOverride", output_guard_file);
    RequireContains(real_content, "RestoreArmRuntimeStateLocked", real_file);
    RequireContains(real_content, "Go2X5ArmOutputGuard::ApplyOutputGuards", real_file);
    RequireContains(operator_content, "real_deploy_exclusive_keyboard_control", operator_file);
    RequireContains(operator_content, "key1_prefer_navigation_mode", operator_file);
    RequireContains(real_content, "key2_prefer_topic_command", real_file);
    RequireContains(sdk_content, "Key[1] pressed: RL policy mode ON (exclusive real deploy control)", sdk_file);
    RequireContains(sdk_content, "this->control.navigation_mode = false;", sdk_file);
    RequireContains(bridge_content, "def _clip_command(", bridge_file);
    RequireContains(bridge_content, "Clip arm cmd to deploy limits", bridge_file);
    RequireContains(bridge_content, "self.state_from_backend = state_from_backend", bridge_file);
    RequireContains(bridge_content, "self.declare_parameter(\"joint_pos_min\", [])", bridge_file);
    RequireContains(base_content, "joint_lower_limits", base_file);
    RequireContains(base_content, "joint_upper_limits", base_file);
    RequireContains(base_content, "joint_velocity_limits", base_file);
    RequireContains(base_content, "arm_command_size: 6", base_file);
    RequireContains(base_content, "arm_joint_command_topic: \"/arm_joint_pos_cmd\"", base_file);
    RequireContains(base_content, "key2_prefer_topic_command: true", base_file);
    RequireContains(base_content, "arm_hold_pose:", base_file);
    RequireContains(base_content, "arm_key_pose:", base_file);
    RequireContains(base_content, "policy_inference_log_enabled: true", base_file);
    RequireNotContains(base_content, "real_deploy_exclusive_keyboard_control: true", base_file);
    RequireContains(base_content, "arm_bridge_require_live_state: true", base_file);
    RequireContains(config_content, "arm_bridge_require_live_state: true", config_file);
    RequireContains(config_content, "arm_bridge_shadow_feedback_enabled: false", config_file);
    RequireContains(config_content, "real_deploy_exclusive_keyboard_control: true", config_file);
    RequireContains(config_content, "policy_inference_log_enabled: true", config_file);
    RequireContains(config_content, "fixed_cmd_x: 0.5", config_file);
    RequireContains(config_content, "fixed_cmd_y: 0.0", config_file);
    RequireContains(config_content, "fixed_cmd_yaw: 0.0", config_file);
    RequireContains(config_content, "key1_prefer_navigation_mode: false", config_file);
    RequireContains(config_content, "key2_prefer_topic_command: true", config_file);
    RequireContains(config_content, "arm_hold_enabled: false", config_file);

    std::cout << "go2_x5 real deploy safety guards test passed." << std::endl;
    return 0;
}
