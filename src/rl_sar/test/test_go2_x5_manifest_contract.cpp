#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "library/core/config/deploy_manifest_loader.hpp"
#include "library/core/config/deploy_manifest_runtime.hpp"

namespace
{

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

void RequireContains(const std::string& content, const std::string& needle, const char* file)
{
    if (content.find(needle) == std::string::npos)
    {
        std::cerr << "Missing expected snippet in " << file << ": " << needle << '\n';
        std::abort();
    }
}

void CheckFrozenManifest(const RLConfig::DeployManifest& manifest)
{
    Require(manifest.meta.manifest_version == 1, "manifest_version must stay at 1");
    Require(manifest.meta.protocol_version == 1, "protocol_version must stay at 1");
    Require(manifest.robot.name == "go2_x5", "robot name mismatch");
    Require(manifest.robot.leg_joint_count == 12, "leg joint count mismatch");
    Require(manifest.robot.arm_joint_count == 6, "arm joint count mismatch");
    Require(!manifest.robot.gripper_enabled, "gripper must stay disabled");
    Require(manifest.policy.mode == "dog_only", "policy mode mismatch");
    Require(manifest.policy.action_dim == 12, "policy action dim mismatch");
    Require(manifest.policy.observation_dim == 260, "policy observation dim mismatch");
    Require(manifest.policy.policy_rate_hz == 50, "policy rate mismatch");
    Require(manifest.policy.enable_arm_tracking_error_obs == false, "tracking error obs must stay disabled");
    Require(manifest.policy.enable_arm_command_delta_obs == false, "command delta obs must stay disabled");
    Require(manifest.body_adapter.command_rate_hz == 200, "body command rate mismatch");
    Require(manifest.body_adapter.lowstate_timeout_ms == 50, "body lowstate timeout mismatch");
    Require(manifest.arm_adapter.arm_target_rate_hz == 200, "arm target rate mismatch");
    Require(manifest.arm_adapter.servo_rate_hz == 500, "arm servo rate mismatch");
    Require(manifest.arm_adapter.require_live_state, "arm live state must stay required");
    Require(manifest.arm_adapter.arm_state_timeout_ms == 50, "arm state timeout mismatch");
    Require(manifest.arm_adapter.arm_cmd_topic == "/arx_x5/joint_cmd", "arm cmd topic mismatch");
    Require(manifest.arm_adapter.arm_state_topic == "/arx_x5/joint_state", "arm state topic mismatch");
    Require(manifest.arm_adapter.arm_joint_command_topic == "/arm_joint_pos_cmd",
        "arm joint command topic mismatch");
    Require(manifest.coordinator.rate_hz == 200, "coordinator rate mismatch");
    Require(manifest.coordinator.body_command_expire_ms == 15, "body command expire mismatch");
    Require(manifest.coordinator.arm_command_expire_ms == 15, "arm command expire mismatch");
    Require(manifest.supervisor.policy_timeout_ms == 100, "supervisor policy timeout mismatch");
    Require(manifest.supervisor.fault_latched_requires_manual_reset, "fault latch reset requirement mismatch");
    Require(manifest.ops.ros2_enabled, "ros2 must stay enabled");
    Require(manifest.ops.ros2_mirror_only, "ros2 mirror only must stay enabled");
    Require(manifest.ops.bridge_rmw_implementation == "rmw_cyclonedds_cpp", "bridge RMW mismatch");
    Require(manifest.ops.go2_rmw_implementation == "rmw_fastrtps_cpp", "runtime RMW mismatch");
    Require(manifest.ops.diagnostics_rate_hz == 50, "diagnostics rate mismatch");
}

void CheckRuntimeSnapshot(const RLConfig::DeployManifestRuntimeSnapshot& snapshot,
    const RLConfig::DeployManifestLoader& loader)
{
    Require(snapshot.manifest_valid, "runtime snapshot should report valid manifest");
    Require(snapshot.manifest_path == loader.ManifestPath(), "runtime snapshot path mismatch");
    Require(snapshot.manifest_hash == loader.ManifestHash(), "runtime snapshot hash mismatch");
    Require(snapshot.policy_id == loader.Manifest().meta.policy_id, "runtime snapshot policy id mismatch");
    Require(snapshot.policy_id_hash != 0, "runtime snapshot policy hash should not be zero");
    Require(snapshot.protocol_version == 1, "runtime snapshot protocol version mismatch");
    Require(snapshot.ros2_enabled, "runtime snapshot ros2_enabled mismatch");
    Require(snapshot.ros2_mirror_only, "runtime snapshot ros2_mirror_only mismatch");
    Require(snapshot.policy_rate_hz == 50, "runtime snapshot policy rate mismatch");
    Require(snapshot.coordinator_rate_hz == 200, "runtime snapshot coordinator rate mismatch");
    Require(snapshot.body_command_rate_hz == 200, "runtime snapshot body command rate mismatch");
    Require(snapshot.arm_target_rate_hz == 200, "runtime snapshot arm target rate mismatch");
    Require(snapshot.arm_servo_rate_hz == 500, "runtime snapshot arm servo rate mismatch");
    Require(snapshot.arm_state_timeout_ms == 50, "runtime snapshot arm state timeout mismatch");
    Require(snapshot.arm_cmd_topic == "/arx_x5/joint_cmd", "runtime snapshot arm cmd topic mismatch");
    Require(snapshot.arm_state_topic == "/arx_x5/joint_state", "runtime snapshot arm state topic mismatch");
    Require(snapshot.arm_joint_command_topic == "/arm_joint_pos_cmd",
        "runtime snapshot arm joint command topic mismatch");
    Require(snapshot.lowstate_timeout_ms == 50, "runtime snapshot lowstate timeout mismatch");
    Require(snapshot.policy_timeout_ms == 100, "runtime snapshot policy timeout mismatch");
    Require(snapshot.body_command_expire_ms == 15, "runtime snapshot body expire mismatch");
    Require(snapshot.arm_command_expire_ms == 15, "runtime snapshot arm expire mismatch");
    Require(snapshot.arm_bridge_transport == "ipc", "runtime snapshot transport mismatch");
    Require(snapshot.bridge_rmw_implementation == "rmw_cyclonedds_cpp", "runtime snapshot bridge RMW mismatch");
    Require(snapshot.go2_rmw_implementation == "rmw_fastrtps_cpp", "runtime snapshot runtime RMW mismatch");
}

} // namespace

int main()
{
    const std::filesystem::path repo_root = "/home/lemon/Issac/rl_ras_n";
    const std::filesystem::path manifest_path = repo_root / RLConfig::DeployManifestLoader::kDefaultManifestPath;
    const std::string manifest_path_str = manifest_path.string();

    Require(std::filesystem::exists(manifest_path), "default manifest file missing");
    Require(std::string(RLConfig::DeployManifestLoader::kDefaultManifestPath) == "deploy/go2_x5_real.yaml",
        "default manifest path changed");

    RLConfig::DeployManifestLoader loader(manifest_path_str);
    const auto load_result = loader.LoadDefaultManifest();
    Require(load_result.is_valid, load_result.error_message.c_str());
    Require(loader.HasManifest(), "loader should retain loaded manifest");
    Require(loader.ManifestPath() == manifest_path_str, "manifest path should be preserved");
    CheckFrozenManifest(loader.Manifest());
    Require(loader.Validate().is_valid, "loaded manifest should validate");

    const std::string hash = loader.ManifestHash();
    Require(!hash.empty(), "manifest hash should not be empty");

    const YAML::Node node = YAML::LoadFile(manifest_path_str);
    RLConfig::DeployManifestLoader loader_from_node(manifest_path_str);
    const auto node_result = loader_from_node.LoadFromNode(node);
    Require(node_result.is_valid, node_result.error_message.c_str());
    Require(loader_from_node.ManifestHash() == hash, "hash should be stable across load paths");

    YAML::Node bad = node;
    bad["policy"]["action_dim"] = 18;
    RLConfig::DeployManifestLoader invalid_loader(manifest_path_str);
    const auto invalid_result = invalid_loader.LoadFromNode(bad);
    Require(!invalid_result.is_valid, "invalid action_dim should be rejected");
    Require(invalid_result.error_path.find("policy.action_dim") != std::string::npos,
        "invalid error path should point at policy.action_dim");

    const std::string manifest_dump = loader.ManifestDump();
    RequireContains(manifest_dump, "policy.action_dim=12", manifest_path_str.c_str());
    RequireContains(manifest_dump, "coordinator.rate_hz=200", manifest_path_str.c_str());
    RequireContains(manifest_dump, "arm_adapter.arm_cmd_topic=/arx_x5/joint_cmd", manifest_path_str.c_str());
    RequireContains(manifest_dump, "ops.bridge_rmw_implementation=rmw_cyclonedds_cpp", manifest_path_str.c_str());

    RLConfig::DeployManifestRuntime runtime(manifest_path_str);
    const auto runtime_result = runtime.LoadFromFile(manifest_path_str);
    Require(runtime_result.is_valid, runtime_result.error_message.c_str());
    CheckRuntimeSnapshot(runtime.Snapshot(), runtime.Loader());

    std::cout << "test_go2_x5_manifest_contract passed\n";
    return 0;
}
