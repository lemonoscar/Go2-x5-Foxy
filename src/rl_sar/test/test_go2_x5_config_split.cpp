#include <cassert>
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
    const std::string base_file = source_dir + "/../../policy/go2_x5/base.yaml";
    const std::string config_file = source_dir + "/../../policy/go2_x5/robot_lab/config.yaml";

    const std::string base_content = ReadAll(base_file);
    const std::string config_content = ReadAll(config_file);

    RequireContains(base_content, "num_of_dofs: 18", base_file);
    RequireContains(base_content, "default_dof_pos:", base_file);
    RequireContains(base_content, "joint_lower_limits:", base_file);
    RequireContains(base_content, "joint_upper_limits:", base_file);
    RequireContains(base_content, "joint_velocity_limits:", base_file);
    RequireContains(base_content, "arm_control_mode: \"split\"", base_file);
    RequireContains(base_content, "arm_bridge_require_state: true", base_file);
    RequireContains(base_content, "arm_bridge_require_live_state: true", base_file);
    RequireContains(base_content, "arm_hold_enabled: true", base_file);
    RequireNotContains(base_content, "real_deploy_exclusive_keyboard_control:", base_file);
    RequireNotContains(base_content, "key2_prefer_topic_command:", base_file);
    RequireNotContains(base_content, "cmd_vel_alpha:", base_file);
    RequireNotContains(base_content, "key1_prefer_navigation_mode:", base_file);
    RequireNotContains(base_content, "key1_publish_cmd_vel_on_navigation:", base_file);
    RequireNotContains(base_content, "key1_navigation_cmd_x:", base_file);
    RequireNotContains(base_content, "fixed_cmd_x:", base_file);

    RequireContains(config_content, "model_name: \"policy.onnx\"", config_file);
    RequireContains(config_content, "num_observations: 260", config_file);
    RequireContains(config_content, "observations: [\"lin_vel\", \"ang_vel\", \"gravity_vec\", \"commands\", \"dof_pos\", \"dof_vel\", \"actions\", \"height_scan\", \"arm_joint_command\", \"gripper_command\"]", config_file);
    RequireContains(config_content, "fixed_cmd_x: 0.5", config_file);
    RequireContains(config_content, "fixed_cmd_y: 0.0", config_file);
    RequireContains(config_content, "fixed_cmd_yaw: 0.0", config_file);
    RequireContains(config_content, "real_deploy_exclusive_keyboard_control: true", config_file);
    RequireContains(config_content, "arm_control_mode: \"split\"", config_file);
    RequireContains(config_content, "arm_bridge_require_state: true", config_file);
    RequireContains(config_content, "arm_bridge_require_live_state: true", config_file);
    RequireContains(config_content, "arm_bridge_shadow_feedback_enabled: false", config_file);
    RequireContains(config_content, "arm_hold_enabled: false", config_file);
    RequireContains(config_content, "arm_hold_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]", config_file);
    RequireContains(config_content, "arm_key_pose: [0, 1.57, 1.57, 0.0, 0.0, 0.0]", config_file);
    RequireNotContains(config_content, "cmd_vel_alpha:", config_file);
    RequireNotContains(config_content, "key1_prefer_navigation_mode:", config_file);
    RequireNotContains(config_content, "key1_publish_cmd_vel_on_navigation:", config_file);
    RequireNotContains(config_content, "key1_navigation_cmd_x:", config_file);
    RequireNotContains(config_content, "key1_navigation_cmd_y:", config_file);
    RequireNotContains(config_content, "key1_navigation_cmd_yaw:", config_file);
    RequireNotContains(config_content, "key2_prefer_topic_command:", config_file);

    std::cout << "go2_x5 config split test passed." << std::endl;
    return 0;
}
