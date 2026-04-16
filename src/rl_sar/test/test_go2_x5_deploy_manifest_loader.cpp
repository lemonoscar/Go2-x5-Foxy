#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "library/core/config/deploy_manifest_loader.hpp"

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

void CheckFrozenContract(const RLConfig::DeployManifest& manifest)
{
    Require(manifest.policy.action_dim == 12, "policy.action_dim must be 12");
    Require(manifest.policy.observation_dim == 260, "policy.observation_dim must be 260");
    Require(manifest.policy.policy_rate_hz == 50, "policy.policy_rate_hz must be 50");
    Require(manifest.body_adapter.command_rate_hz == 200, "body_adapter.command_rate_hz must be 200");
    Require(manifest.body_adapter.lowstate_timeout_ms == 50, "body_adapter.lowstate_timeout_ms must be 50");
    Require(manifest.body_adapter.enable_velocity_estimation, "body_adapter.enable_velocity_estimation must be true");
    Require(std::abs(manifest.body_adapter.velocity_estimator.hip_length - 0.08) < 1e-9,
        "body_adapter.velocity_estimator.hip_length mismatch");
    Require(std::abs(manifest.body_adapter.velocity_estimator.thigh_length - 0.213) < 1e-9,
        "body_adapter.velocity_estimator.thigh_length mismatch");
    Require(std::abs(manifest.body_adapter.velocity_estimator.calf_length - 0.213) < 1e-9,
        "body_adapter.velocity_estimator.calf_length mismatch");
    Require(manifest.body_adapter.velocity_estimator.moving_window_size == 120,
        "body_adapter.velocity_estimator.moving_window_size mismatch");
    Require(std::abs(manifest.body_adapter.velocity_estimator.foot_contact_force_threshold - 20.0) < 1e-9,
        "body_adapter.velocity_estimator.foot_contact_force_threshold mismatch");
    Require(manifest.arm_adapter.arm_target_rate_hz == 200, "arm_adapter.arm_target_rate_hz must be 200");
    Require(manifest.arm_adapter.servo_rate_hz == 500, "arm_adapter.servo_rate_hz must be 500");
    Require(manifest.arm_adapter.arm_state_timeout_ms == 50, "arm_adapter.arm_state_timeout_ms must be 50");
    Require(manifest.arm_adapter.arm_cmd_topic == "/arx_x5/joint_cmd", "arm_adapter.arm_cmd_topic mismatch");
    Require(manifest.arm_adapter.arm_state_topic == "/arx_x5/joint_state", "arm_adapter.arm_state_topic mismatch");
    Require(manifest.arm_adapter.arm_joint_command_topic == "/arm_joint_pos_cmd",
        "arm_adapter.arm_joint_command_topic mismatch");
    Require(manifest.coordinator.rate_hz == 200, "coordinator.rate_hz must be 200");
    Require(manifest.coordinator.body_command_expire_ms == 15, "coordinator.body_command_expire_ms must be 15");
    Require(manifest.coordinator.arm_command_expire_ms == 15, "coordinator.arm_command_expire_ms must be 15");
    Require(manifest.supervisor.policy_timeout_ms == 100, "supervisor.policy_timeout_ms must be 100");
    Require(manifest.ops.bridge_rmw_implementation == "rmw_cyclonedds_cpp",
        "ops.bridge_rmw_implementation mismatch");
    Require(manifest.ops.go2_rmw_implementation == "rmw_fastrtps_cpp",
        "ops.go2_rmw_implementation mismatch");
    Require(manifest.ops.diagnostics_rate_hz == 50, "ops.diagnostics_rate_hz must be 50");
    Require(!manifest.startup_sequence.enabled, "startup_sequence.enabled must default to false");
    Require(std::abs(manifest.startup_sequence.get_up_delay_sec - 5.0) < 1e-9,
        "startup_sequence.get_up_delay_sec mismatch");
    Require(std::abs(manifest.startup_sequence.rl_delay_after_get_up_sec - 5.0) < 1e-9,
        "startup_sequence.rl_delay_after_get_up_sec mismatch");
}

} // namespace

int main()
{
    const std::string manifest_file = "/home/lemon/Issac/rl_ras_n/deploy/go2_x5_real.yaml";
    Require(std::filesystem::exists(manifest_file), "manifest file missing");

    RLConfig::DeployManifestLoader loader(manifest_file);
    const auto load_default = loader.LoadDefaultManifest();
    Require(load_default.is_valid, load_default.error_message.c_str());
    Require(loader.HasManifest(), "loader should retain the manifest");
    Require(loader.ManifestPath() == manifest_file, "manifest path should be preserved");
    CheckFrozenContract(loader.Manifest());
    Require(loader.Validate().is_valid, "loaded manifest should validate");
    const std::string hash = loader.ManifestHash();
    Require(!hash.empty(), "manifest hash should not be empty");

    const auto load_file = loader.LoadFromFile(manifest_file);
    Require(load_file.is_valid, load_file.error_message.c_str());
    Require(loader.ManifestHash() == hash, "hash should remain stable across file load");

    const YAML::Node node = YAML::LoadFile(manifest_file);
    const auto load_node = loader.LoadFromNode(node);
    Require(load_node.is_valid, load_node.error_message.c_str());
    Require(loader.ManifestHash() == hash, "hash should remain stable across node load");

    YAML::Node invalid = node;
    invalid["robot"]["arm_joint_count"] = 8;
    const auto invalid_result = loader.LoadFromNode(invalid);
    Require(!invalid_result.is_valid, "invalid arm joint count should be rejected");
    Require(invalid_result.error_path.find("robot.arm_joint_count") != std::string::npos,
        "invalid arm joint count should point at robot.arm_joint_count");

    YAML::Node invalid_rmw = node;
    invalid_rmw["ops"]["bridge_rmw_implementation"] = "rmw_fastrtps_cpp";
    RLConfig::DeployManifestLoader invalid_rmw_loader(manifest_file);
    const auto invalid_rmw_result = invalid_rmw_loader.LoadFromNode(invalid_rmw);
    Require(!invalid_rmw_result.is_valid, "invalid bridge RMW should be rejected");
    Require(
        !invalid_rmw_result.error_path.empty() || !invalid_rmw_result.error_message.empty(),
        "invalid bridge RMW should surface an error");

    std::cout << "test_go2_x5_deploy_manifest_loader passed\n";
    return 0;
}
