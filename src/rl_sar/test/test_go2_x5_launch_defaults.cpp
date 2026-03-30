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

}  // namespace

int main()
{
    const std::string source_dir = RL_SAR_SOURCE_DIR;
    const std::string launch_file = source_dir + "/launch/go2_x5_real_dual.launch.py";
    const std::string real_file = source_dir + "/src/rl_sar/core/rl_real_go2_x5.cpp";
    const std::string state_io_file = source_dir + "/src/rl_sar/go2x5/state/go2_x5_state_io.cpp";
    const std::string safety_file = source_dir + "/src/rl_sar/go2x5/safety/go2_x5_safety_guard.cpp";
    const std::string config_file = source_dir + "/../../policy/go2_x5/robot_lab/config.yaml";

    const std::string launch_content = ReadAll(launch_file);
    const std::string real_content = ReadAll(real_file);
    const std::string state_io_content = ReadAll(state_io_file);
    const std::string safety_content = ReadAll(safety_file);
    const std::string config_content = ReadAll(config_file);

    RequireContains(launch_content, "DeclareLaunchArgument(\"network_interface\", default_value=\"eth0\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"start_arm_bridge\", default_value=\"true\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_cmd_topic\", default_value=\"/arx_x5/joint_cmd\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_state_topic\", default_value=\"/arx_x5/joint_state\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_joint_command_topic\", default_value=\"/arm_joint_pos_cmd\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_publish_rate_hz\", default_value=\"200.0\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_command_speed\", default_value=\"0.4\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_cmd_timeout_sec\", default_value=\"0.5\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_require_sdk\", default_value=\"true\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_require_initial_state\", default_value=\"true\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_probe_backend_before_init\", default_value=\"true\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_probe_timeout_sec\", default_value=\"5.0\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_enable_background_send_recv\", default_value=\"false\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_controller_dt\", default_value=\"0.002\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_init_to_home\", default_value=\"false\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_ipc_enabled\", default_value=\"true\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_ipc_cmd_port\", default_value=\"45671\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_ipc_state_port\", default_value=\"45672\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_topic_ipc_enabled\", default_value=\"true\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_topic_ipc_port\", default_value=\"45673\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"go2_enable_ros2_runtime\", default_value=\"false\")", launch_file);
    RequireContains(launch_content, "DeclareLaunchArgument(\"go2_arm_bridge_transport\", default_value=\"ipc\")", launch_file);
    RequireContains(launch_content, "\"--arm-bridge-transport\"", launch_file);
    RequireContains(launch_content, "\"--arm-joint-cmd-port\"", launch_file);
    RequireContains(launch_content, "on_exit=Shutdown(reason=\"arx_x5_bridge exited\")", launch_file);

    RequireContains(config_content, "real_deploy_exclusive_keyboard_control: true", config_file);
    RequireContains(config_content, "key1_prefer_navigation_mode: false", config_file);
    RequireContains(config_content, "key2_prefer_topic_command: true", config_file);
    RequireContains(config_content, "arm_bridge_shadow_feedback_enabled: false", config_file);
    RequireContains(config_content, "arm_hold_enabled: false", config_file);
    RequireContains(config_content, "arm_bridge_require_state: true", config_file);
    RequireContains(config_content, "arm_bridge_require_live_state: true", config_file);
    RequireContains(config_content, "fixed_cmd_x: 0.5", config_file);
    RequireContains(config_content, "fixed_cmd_y: 0.0", config_file);
    RequireContains(config_content, "fixed_cmd_yaw: 0.0", config_file);

    RequireContains(safety_content, "state_snapshot.require_live_state && !state_snapshot.state_from_backend", safety_file);
    RequireContains(state_io_content, "Ignoring shadow arm feedback until a real backend state arrives.", state_io_file);
    RequireContains(state_io_content, "Ignoring shadow arm feedback until bridge recovers.", state_io_file);
    RequireContains(state_io_content, "shadow-only state", state_io_file);

    std::cout << "go2_x5 launch defaults test passed." << std::endl;
    return 0;
}
