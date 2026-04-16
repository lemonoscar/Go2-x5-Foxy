#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "library/core/config/deploy_manifest_loader.hpp"

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
    RequireContains(launch_content, "def _resolve_manifest_path(path: str) -> str:", launch_file.string());
    RequireContains(launch_content, "\"deploy\" / \"go2_x5_real.yaml\"", launch_file.string());
    RequireContains(launch_content, "Path(path).expanduser().resolve()", launch_file.string());
    RequireContains(launch_content, "OpaqueFunction(function=_build_launch_actions)", launch_file.string());
    RequireContains(launch_content, "\"--manifest-path\"", launch_file.string());
    RequireContains(launch_content, "resolved_manifest_path", launch_file.string());
    RequireContains(launch_content, "DeclareLaunchArgument(\"go2_enable_ros2_runtime\"", launch_file.string());
    RequireContains(launch_content, "go2_rmw_implementation", launch_file.string());
    Require(launch_content.find("start_arm_bridge") == std::string::npos,
        "start_arm_bridge launch argument should be removed");
    Require(launch_content.find("go2_arm_bridge_transport_value") == std::string::npos,
        "bridge transport launch defaults should be removed");
    Require(launch_content.find("bridge_rmw_implementation_value") == std::string::npos,
        "bridge RMW launch defaults should be removed");
    RequireContains(launch_content, "go2_x5_node", launch_file.string());
    Require(launch_content.find("arx_x5_bridge") == std::string::npos,
        "arx_x5_bridge node should be removed");
    RequireContains(real_content, "this->InitializeRuntimeOptions(argc, argv);", real_file.string());
    RequireContains(real_content, "this->InitializeConfigLoader();", real_file.string());
    RequireContains(real_content, "deploy_manifest_runtime_->LoadFromFile(this->deploy_manifest_path_)", real_file.string());
    Require(real_content.find("this->InitializeRuntimeOptions(argc, argv);") <
                real_content.find("this->InitializeConfigLoader();"),
        "runtime options must be initialized before config loader");

    std::cout << "go2_x5 launch defaults test passed." << std::endl;
    return 0;
}
