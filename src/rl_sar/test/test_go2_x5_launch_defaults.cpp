#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "deploy_manifest_loader.hpp"

namespace
{

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

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << std::endl;
        std::abort();
    }
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
    const std::filesystem::path repo_root = "/home/lemon/Issac/rl_ras_n";
    const std::filesystem::path launch_file = source_dir + "/launch/go2_x5_real_dual.launch.py";
    const std::filesystem::path real_file = source_dir + "/src/rl_sar/core/rl_real_go2_x5.cpp";
    const std::filesystem::path manifest_file = repo_root / RLConfig::DeployManifestLoader::kDefaultManifestPath;

    Require(std::filesystem::exists(launch_file), "launch file missing");
    Require(std::filesystem::exists(real_file), "rl_real_go2_x5.cpp missing");
    Require(std::filesystem::exists(manifest_file), "default manifest file missing");
    Require(std::string(RLConfig::DeployManifestLoader::kDefaultManifestPath) == "deploy/go2_x5_real.yaml",
        "default manifest path changed");

    const std::string launch_content = ReadAll(launch_file.string());
    const std::string real_content = ReadAll(real_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"deploy_manifest_path\", default_value=_default_manifest_path())", launch_file.string());
    RequireContains(launch_content, "def _default_manifest_path() -> str:", launch_file.string());
    RequireContains(launch_content, "\"deploy\" / \"go2_x5_real.yaml\"", launch_file.string());
    RequireContains(launch_content, "OpaqueFunction(function=_build_launch_actions)", launch_file.string());
    RequireContains(launch_content, "\"--manifest-path\"", launch_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"go2_enable_ros2_runtime\"", launch_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"go2_arm_bridge_transport\"", launch_file.string());
    RequireContains(launch_content, "bridge_rmw_implementation", launch_file.string());
    RequireContains(launch_content, "go2_rmw_implementation", launch_file.string());
    RequireContains(launch_content, "\"arm_cmd_topic\": str(arm_adapter.get(\"arm_cmd_topic\", \"/arx_x5/joint_cmd\"))", launch_file.string());
    RequireContains(launch_content, "\"arm_state_topic\": str(arm_adapter.get(\"arm_state_topic\", \"/arx_x5/joint_state\"))", launch_file.string());
    RequireContains(launch_content, "\"arm_joint_command_topic\": str(arm_adapter.get(\"arm_joint_command_topic\", \"/arm_joint_pos_cmd\"))", launch_file.string());
    RequireContains(launch_content, "\"bridge_rmw_implementation\": str(ops.get(\"bridge_rmw_implementation\", \"rmw_cyclonedds_cpp\"))", launch_file.string());
    RequireContains(launch_content, "\"go2_rmw_implementation\": str(ops.get(\"go2_rmw_implementation\", \"rmw_fastrtps_cpp\"))", launch_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_cmd_topic\", default_value=arm_cmd_topic_value)", launch_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_state_topic\", default_value=arm_state_topic_value)", launch_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"arm_joint_command_topic\", default_value=arm_joint_command_topic_value)", launch_file.string());
    RequireContains(launch_content, "default_value=bridge_rmw_implementation_value", launch_file.string());
    RequireContains(launch_content, "default_value=go2_rmw_implementation_value", launch_file.string());
    RequireContains(launch_content, "\"ipc_bind_host\": go2_arm_bridge_ipc_host", launch_file.string());
    RequireContains(launch_content, "\"ipc_cmd_port\": go2_arm_bridge_cmd_port", launch_file.string());
    RequireContains(launch_content, "\"ipc_state_host\": go2_arm_bridge_ipc_host", launch_file.string());
    RequireContains(launch_content, "\"ipc_state_port\": go2_arm_bridge_state_port", launch_file.string());
    Require(launch_content.find("DeclareLaunchArgument(\"arm_ipc_bind_host\"") == std::string::npos,
        "arm_ipc_bind_host should no longer be a separate launch source");
    Require(launch_content.find("DeclareLaunchArgument(\"arm_ipc_cmd_port\"") == std::string::npos,
        "arm_ipc_cmd_port should no longer be a separate launch source");
    Require(launch_content.find("DeclareLaunchArgument(\"arm_ipc_state_host\"") == std::string::npos,
        "arm_ipc_state_host should no longer be a separate launch source");
    Require(launch_content.find("DeclareLaunchArgument(\"arm_ipc_state_port\"") == std::string::npos,
        "arm_ipc_state_port should no longer be a separate launch source");
    RequireContains(real_content, "this->InitializeRuntimeOptions(argc, argv);", real_file.string());
    RequireContains(real_content, "this->InitializeConfigLoader();", real_file.string());
    RequireContains(real_content, "deploy_manifest_runtime_->LoadFromFile(this->deploy_manifest_path_)", real_file.string());
    RequireContains(real_content, "this->runtime_ros2_enabled_explicit_ = options.enable_ros2_runtime_explicit;", real_file.string());
    RequireContains(real_content, "this->runtime_arm_bridge_transport_explicit_ = options.arm_bridge_transport_explicit;", real_file.string());
    RequireContains(real_content, "if (!this->runtime_ros2_enabled_explicit_)", real_file.string());
    RequireContains(real_content, "this->enable_ros2_runtime = snapshot.ros2_enabled;", real_file.string());
    RequireContains(real_content, "if (!this->runtime_arm_bridge_transport_explicit_)", real_file.string());
    RequireContains(real_content, "this->arm_bridge_transport = snapshot.arm_bridge_transport;", real_file.string());
    Require(real_content.find("this->InitializeRuntimeOptions(argc, argv);") <
                real_content.find("this->InitializeConfigLoader();"),
        "runtime options must be initialized before config loader");

    std::cout << "go2_x5 launch defaults test passed." << std::endl;
    return 0;
}
