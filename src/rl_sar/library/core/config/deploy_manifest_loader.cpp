#include "library/core/config/deploy_manifest_loader.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace RLConfig
{

using ValidationResult = DeployManifestValidationResult;

namespace
{

std::string PathJoin(const std::string& parent, const char* child)
{
    if (parent.empty())
    {
        return child;
    }
    return parent + "." + child;
}

} // namespace

DeployManifestLoader::DeployManifestLoader(std::string manifest_path)
    : manifest_path_(std::move(manifest_path))
{
}

ValidationResult DeployManifestLoader::LoadDefaultManifest()
{
    return this->LoadFromFile(this->manifest_path_);
}

ValidationResult DeployManifestLoader::LoadFromFile(const std::string& manifest_path)
{
    try
    {
        const YAML::Node node = YAML::LoadFile(manifest_path);
        const ValidationResult validation = this->LoadFromNode(node);
        if (validation.is_valid)
        {
            this->manifest_path_ = manifest_path;
        }
        return validation;
    }
    catch (const YAML::Exception& e)
    {
        return ValidationResult::Error(
            std::string("failed to load deploy manifest: ") + e.what(),
            manifest_path);
    }
    catch (const std::exception& e)
    {
        return ValidationResult::Error(
            std::string("failed to load deploy manifest: ") + e.what(),
            manifest_path);
    }
}

ValidationResult DeployManifestLoader::LoadFromNode(const YAML::Node& node)
{
    const ValidationResult validation = ValidateNode(node);
    if (!validation.is_valid)
    {
        this->loaded_ = false;
        return validation;
    }

    this->manifest_ = ParseManifest(node);
    const ValidationResult manifest_validation = ValidateManifest(this->manifest_);
    if (!manifest_validation.is_valid)
    {
        this->loaded_ = false;
        return manifest_validation;
    }

    this->loaded_ = true;
    return manifest_validation;
}

bool DeployManifestLoader::HasManifest() const
{
    return this->loaded_;
}

const std::string& DeployManifestLoader::ManifestPath() const
{
    return this->manifest_path_;
}

const DeployManifest& DeployManifestLoader::Manifest() const
{
    return this->manifest_;
}

ValidationResult DeployManifestLoader::Validate() const
{
    if (!this->loaded_)
    {
        return ValidationResult::Error("deploy manifest has not been loaded");
    }
    return ValidateManifest(this->manifest_);
}

std::string DeployManifestLoader::ManifestDump() const
{
    if (!this->loaded_)
    {
        return {};
    }
    return SerializeManifest(this->manifest_, true);
}

std::string DeployManifestLoader::ManifestHash() const
{
    if (!this->loaded_)
    {
        return {};
    }
    const std::string canonical = SerializeManifest(this->manifest_, false);
    std::ostringstream oss;
    oss << std::hex << std::setw(16) << std::setfill('0') << Fnv1a64(canonical);
    return oss.str();
}

ValidationResult DeployManifestLoader::ValidateNode(const YAML::Node& node)
{
    ValidationResult result = ValidationResult::Ok();
    RequireMap(node, "root", &result);
    if (!result.is_valid)
    {
        return result;
    }

    RequireAllowedKeys(node, {
        "meta", "robot", "policy", "body_adapter", "arm_adapter",
        "coordinator", "supervisor", "ops"
    }, "root", &result);
    if (!result.is_valid)
    {
        return result;
    }

    const YAML::Node meta = node["meta"];
    const YAML::Node robot = node["robot"];
    const YAML::Node policy = node["policy"];
    const YAML::Node body_adapter = node["body_adapter"];
    const YAML::Node arm_adapter = node["arm_adapter"];
    const YAML::Node coordinator = node["coordinator"];
    const YAML::Node supervisor = node["supervisor"];
    const YAML::Node ops = node["ops"];

    RequireMap(meta, "meta", &result);
    RequireMap(robot, "robot", &result);
    RequireMap(policy, "policy", &result);
    RequireMap(body_adapter, "body_adapter", &result);
    RequireMap(arm_adapter, "arm_adapter", &result);
    RequireMap(coordinator, "coordinator", &result);
    RequireMap(supervisor, "supervisor", &result);
    RequireMap(ops, "ops", &result);
    if (!result.is_valid)
    {
        return result;
    }

    RequireAllowedKeys(meta, {"manifest_version", "protocol_version", "policy_id", "generated_at"}, "meta", &result);
    RequireAllowedKeys(robot, {"name", "leg_joint_count", "arm_joint_count", "gripper_enabled",
        "joint_mapping", "joint_lower_limits", "joint_upper_limits", "joint_velocity_limits"}, "robot", &result);
    RequireAllowedKeys(policy, {"mode", "action_dim", "observation_dim", "policy_rate_hz",
        "model_path", "enable_arm_tracking_error_obs", "enable_arm_command_delta_obs"}, "policy", &result);
    RequireAllowedKeys(body_adapter, {"interface", "network_interface", "command_rate_hz",
        "require_lowstate", "lowstate_timeout_ms"}, "body_adapter", &result);
    RequireAllowedKeys(arm_adapter, {"interface", "can_interface", "arm_cmd_topic", "arm_state_topic",
        "arm_joint_command_topic", "arm_target_rate_hz", "servo_rate_hz", "background_send_recv",
        "controller_dt", "require_live_state", "arm_state_timeout_ms", "arm_tracking_error_limit"},
        "arm_adapter", &result);
    RequireAllowedKeys(coordinator, {"rate_hz", "body_command_expire_ms", "arm_command_expire_ms",
        "zero_cmd_xy_drift_limit", "zero_cmd_yaw_drift_limit", "enable_drift_anchor"}, "coordinator", &result);
    RequireAllowedKeys(supervisor, {"probe_window_sec", "degraded_timeout_sec", "policy_timeout_ms",
        "fault_latched_requires_manual_reset"}, "supervisor", &result);
    RequireAllowedKeys(ops, {"ros2_enabled", "ros2_mirror_only", "bridge_rmw_implementation",
        "go2_rmw_implementation", "bag_enabled", "diagnostics_rate_hz"}, "ops", &result);
    if (!result.is_valid)
    {
        return result;
    }

    RequireScalarInt(meta, "manifest_version", "meta", nullptr, &result);
    RequireScalarInt(meta, "protocol_version", "meta", nullptr, &result);
    RequireScalarString(meta, "policy_id", "meta", nullptr, &result);
    RequireScalarString(meta, "generated_at", "meta", nullptr, &result);

    RequireScalarString(robot, "name", "robot", nullptr, &result);
    RequireScalarInt(robot, "leg_joint_count", "robot", nullptr, &result);
    RequireScalarInt(robot, "arm_joint_count", "robot", nullptr, &result);
    RequireScalarBool(robot, "gripper_enabled", "robot", nullptr, &result);
    RequireIntVector(robot, "joint_mapping", 18, "robot", nullptr, &result);
    RequireDoubleVector(robot, "joint_lower_limits", 18, "robot", nullptr, &result);
    RequireDoubleVector(robot, "joint_upper_limits", 18, "robot", nullptr, &result);
    RequireDoubleVector(robot, "joint_velocity_limits", 18, "robot", nullptr, &result);

    RequireScalarString(policy, "mode", "policy", nullptr, &result);
    RequireScalarInt(policy, "action_dim", "policy", nullptr, &result);
    RequireScalarInt(policy, "observation_dim", "policy", nullptr, &result);
    RequireScalarInt(policy, "policy_rate_hz", "policy", nullptr, &result);
    RequireScalarString(policy, "model_path", "policy", nullptr, &result);
    RequireScalarBool(policy, "enable_arm_tracking_error_obs", "policy", nullptr, &result);
    RequireScalarBool(policy, "enable_arm_command_delta_obs", "policy", nullptr, &result);

    RequireScalarString(body_adapter, "interface", "body_adapter", nullptr, &result);
    RequireScalarString(body_adapter, "network_interface", "body_adapter", nullptr, &result);
    RequireScalarInt(body_adapter, "command_rate_hz", "body_adapter", nullptr, &result);
    RequireScalarBool(body_adapter, "require_lowstate", "body_adapter", nullptr, &result);
    RequireScalarInt(body_adapter, "lowstate_timeout_ms", "body_adapter", nullptr, &result);

    RequireScalarString(arm_adapter, "interface", "arm_adapter", nullptr, &result);
    RequireScalarString(arm_adapter, "can_interface", "arm_adapter", nullptr, &result);
    RequireScalarString(arm_adapter, "arm_cmd_topic", "arm_adapter", nullptr, &result);
    RequireScalarString(arm_adapter, "arm_state_topic", "arm_adapter", nullptr, &result);
    RequireScalarString(arm_adapter, "arm_joint_command_topic", "arm_adapter", nullptr, &result);
    RequireScalarInt(arm_adapter, "arm_target_rate_hz", "arm_adapter", nullptr, &result);
    RequireScalarInt(arm_adapter, "servo_rate_hz", "arm_adapter", nullptr, &result);
    RequireScalarBool(arm_adapter, "background_send_recv", "arm_adapter", nullptr, &result);
    RequireScalarDouble(arm_adapter, "controller_dt", "arm_adapter", nullptr, &result);
    RequireScalarBool(arm_adapter, "require_live_state", "arm_adapter", nullptr, &result);
    RequireScalarInt(arm_adapter, "arm_state_timeout_ms", "arm_adapter", nullptr, &result);
    RequireScalarDouble(arm_adapter, "arm_tracking_error_limit", "arm_adapter", nullptr, &result);

    RequireScalarInt(coordinator, "rate_hz", "coordinator", nullptr, &result);
    RequireScalarInt(coordinator, "body_command_expire_ms", "coordinator", nullptr, &result);
    RequireScalarInt(coordinator, "arm_command_expire_ms", "coordinator", nullptr, &result);
    RequireScalarDouble(coordinator, "zero_cmd_xy_drift_limit", "coordinator", nullptr, &result);
    RequireScalarDouble(coordinator, "zero_cmd_yaw_drift_limit", "coordinator", nullptr, &result);
    RequireScalarBool(coordinator, "enable_drift_anchor", "coordinator", nullptr, &result);

    RequireScalarDouble(supervisor, "probe_window_sec", "supervisor", nullptr, &result);
    RequireScalarDouble(supervisor, "degraded_timeout_sec", "supervisor", nullptr, &result);
    RequireScalarInt(supervisor, "policy_timeout_ms", "supervisor", nullptr, &result);
    RequireScalarBool(supervisor, "fault_latched_requires_manual_reset", "supervisor", nullptr, &result);

    RequireScalarBool(ops, "ros2_enabled", "ops", nullptr, &result);
    RequireScalarBool(ops, "ros2_mirror_only", "ops", nullptr, &result);
    RequireScalarString(ops, "bridge_rmw_implementation", "ops", nullptr, &result);
    RequireScalarString(ops, "go2_rmw_implementation", "ops", nullptr, &result);
    RequireScalarBool(ops, "bag_enabled", "ops", nullptr, &result);
    RequireScalarInt(ops, "diagnostics_rate_hz", "ops", nullptr, &result);
    return result;
}

DeployManifest DeployManifestLoader::ParseManifest(const YAML::Node& node)
{
    DeployManifest manifest;
    const YAML::Node meta = node["meta"];
    const YAML::Node robot = node["robot"];
    const YAML::Node policy = node["policy"];
    const YAML::Node body_adapter = node["body_adapter"];
    const YAML::Node arm_adapter = node["arm_adapter"];
    const YAML::Node coordinator = node["coordinator"];
    const YAML::Node supervisor = node["supervisor"];
    const YAML::Node ops = node["ops"];

    RequireScalarInt(meta, "manifest_version", "meta", &manifest.meta.manifest_version, nullptr);
    RequireScalarInt(meta, "protocol_version", "meta", &manifest.meta.protocol_version, nullptr);
    RequireScalarString(meta, "policy_id", "meta", &manifest.meta.policy_id, nullptr);
    RequireScalarString(meta, "generated_at", "meta", &manifest.meta.generated_at, nullptr);

    RequireScalarString(robot, "name", "robot", &manifest.robot.name, nullptr);
    RequireScalarInt(robot, "leg_joint_count", "robot", &manifest.robot.leg_joint_count, nullptr);
    RequireScalarInt(robot, "arm_joint_count", "robot", &manifest.robot.arm_joint_count, nullptr);
    RequireScalarBool(robot, "gripper_enabled", "robot", &manifest.robot.gripper_enabled, nullptr);
    RequireIntVector(robot, "joint_mapping", 18, "robot", &manifest.robot.joint_mapping, nullptr);
    RequireDoubleVector(robot, "joint_lower_limits", 18, "robot", &manifest.robot.joint_lower_limits, nullptr);
    RequireDoubleVector(robot, "joint_upper_limits", 18, "robot", &manifest.robot.joint_upper_limits, nullptr);
    RequireDoubleVector(robot, "joint_velocity_limits", 18, "robot", &manifest.robot.joint_velocity_limits, nullptr);

    RequireScalarString(policy, "mode", "policy", &manifest.policy.mode, nullptr);
    RequireScalarInt(policy, "action_dim", "policy", &manifest.policy.action_dim, nullptr);
    RequireScalarInt(policy, "observation_dim", "policy", &manifest.policy.observation_dim, nullptr);
    RequireScalarInt(policy, "policy_rate_hz", "policy", &manifest.policy.policy_rate_hz, nullptr);
    RequireScalarString(policy, "model_path", "policy", &manifest.policy.model_path, nullptr);
    RequireScalarBool(policy, "enable_arm_tracking_error_obs", "policy", &manifest.policy.enable_arm_tracking_error_obs, nullptr);
    RequireScalarBool(policy, "enable_arm_command_delta_obs", "policy", &manifest.policy.enable_arm_command_delta_obs, nullptr);

    RequireScalarString(body_adapter, "interface", "body_adapter", &manifest.body_adapter.interface, nullptr);
    RequireScalarString(body_adapter, "network_interface", "body_adapter", &manifest.body_adapter.network_interface, nullptr);
    RequireScalarInt(body_adapter, "command_rate_hz", "body_adapter", &manifest.body_adapter.command_rate_hz, nullptr);
    RequireScalarBool(body_adapter, "require_lowstate", "body_adapter", &manifest.body_adapter.require_lowstate, nullptr);
    RequireScalarInt(body_adapter, "lowstate_timeout_ms", "body_adapter", &manifest.body_adapter.lowstate_timeout_ms, nullptr);

    RequireScalarString(arm_adapter, "interface", "arm_adapter", &manifest.arm_adapter.interface, nullptr);
    RequireScalarString(arm_adapter, "can_interface", "arm_adapter", &manifest.arm_adapter.can_interface, nullptr);
    RequireScalarString(arm_adapter, "arm_cmd_topic", "arm_adapter", &manifest.arm_adapter.arm_cmd_topic, nullptr);
    RequireScalarString(arm_adapter, "arm_state_topic", "arm_adapter", &manifest.arm_adapter.arm_state_topic, nullptr);
    RequireScalarString(arm_adapter, "arm_joint_command_topic", "arm_adapter",
        &manifest.arm_adapter.arm_joint_command_topic, nullptr);
    RequireScalarInt(arm_adapter, "arm_target_rate_hz", "arm_adapter", &manifest.arm_adapter.arm_target_rate_hz, nullptr);
    RequireScalarInt(arm_adapter, "servo_rate_hz", "arm_adapter", &manifest.arm_adapter.servo_rate_hz, nullptr);
    RequireScalarBool(arm_adapter, "background_send_recv", "arm_adapter", &manifest.arm_adapter.background_send_recv, nullptr);
    RequireScalarDouble(arm_adapter, "controller_dt", "arm_adapter", &manifest.arm_adapter.controller_dt, nullptr);
    RequireScalarBool(arm_adapter, "require_live_state", "arm_adapter", &manifest.arm_adapter.require_live_state, nullptr);
    RequireScalarInt(arm_adapter, "arm_state_timeout_ms", "arm_adapter", &manifest.arm_adapter.arm_state_timeout_ms, nullptr);
    RequireScalarDouble(arm_adapter, "arm_tracking_error_limit", "arm_adapter", &manifest.arm_adapter.arm_tracking_error_limit, nullptr);

    RequireScalarInt(coordinator, "rate_hz", "coordinator", &manifest.coordinator.rate_hz, nullptr);
    RequireScalarInt(coordinator, "body_command_expire_ms", "coordinator", &manifest.coordinator.body_command_expire_ms, nullptr);
    RequireScalarInt(coordinator, "arm_command_expire_ms", "coordinator", &manifest.coordinator.arm_command_expire_ms, nullptr);
    RequireScalarDouble(coordinator, "zero_cmd_xy_drift_limit", "coordinator", &manifest.coordinator.zero_cmd_xy_drift_limit, nullptr);
    RequireScalarDouble(coordinator, "zero_cmd_yaw_drift_limit", "coordinator", &manifest.coordinator.zero_cmd_yaw_drift_limit, nullptr);
    RequireScalarBool(coordinator, "enable_drift_anchor", "coordinator", &manifest.coordinator.enable_drift_anchor, nullptr);

    RequireScalarDouble(supervisor, "probe_window_sec", "supervisor", &manifest.supervisor.probe_window_sec, nullptr);
    RequireScalarDouble(supervisor, "degraded_timeout_sec", "supervisor", &manifest.supervisor.degraded_timeout_sec, nullptr);
    RequireScalarInt(supervisor, "policy_timeout_ms", "supervisor", &manifest.supervisor.policy_timeout_ms, nullptr);
    RequireScalarBool(supervisor, "fault_latched_requires_manual_reset", "supervisor", &manifest.supervisor.fault_latched_requires_manual_reset, nullptr);

    RequireScalarBool(ops, "ros2_enabled", "ops", &manifest.ops.ros2_enabled, nullptr);
    RequireScalarBool(ops, "ros2_mirror_only", "ops", &manifest.ops.ros2_mirror_only, nullptr);
    RequireScalarString(ops, "bridge_rmw_implementation", "ops", &manifest.ops.bridge_rmw_implementation, nullptr);
    RequireScalarString(ops, "go2_rmw_implementation", "ops", &manifest.ops.go2_rmw_implementation, nullptr);
    RequireScalarBool(ops, "bag_enabled", "ops", &manifest.ops.bag_enabled, nullptr);
    RequireScalarInt(ops, "diagnostics_rate_hz", "ops", &manifest.ops.diagnostics_rate_hz, nullptr);
    return manifest;
}

ValidationResult DeployManifestLoader::ValidateManifest(const DeployManifest& manifest)
{
    if (manifest.meta.manifest_version <= 0)
    {
        return ValidationResult::Error("manifest_version must be positive", "meta.manifest_version");
    }
    if (manifest.meta.protocol_version != 1)
    {
        return ValidationResult::Error("protocol_version must be 1", "meta.protocol_version");
    }
    if (manifest.meta.policy_id.empty())
    {
        return ValidationResult::Error("policy_id must be non-empty", "meta.policy_id");
    }
    if (manifest.meta.generated_at.empty())
    {
        return ValidationResult::Error("generated_at must be non-empty", "meta.generated_at");
    }

    if (manifest.robot.name != "go2_x5")
    {
        return ValidationResult::Error("robot.name must be go2_x5", "robot.name");
    }
    if (manifest.robot.leg_joint_count != 12)
    {
        return ValidationResult::Error("robot.leg_joint_count must be 12", "robot.leg_joint_count");
    }
    if (manifest.robot.arm_joint_count != 6)
    {
        return ValidationResult::Error("robot.arm_joint_count must be 6", "robot.arm_joint_count");
    }
    if (manifest.robot.gripper_enabled)
    {
        return ValidationResult::Error("robot.gripper_enabled must be false", "robot.gripper_enabled");
    }
    if (manifest.robot.joint_mapping.size() != 18)
    {
        return ValidationResult::Error("robot.joint_mapping must have 18 entries", "robot.joint_mapping");
    }
    if (manifest.robot.joint_lower_limits.size() != 18)
    {
        return ValidationResult::Error("robot.joint_lower_limits must have 18 entries", "robot.joint_lower_limits");
    }
    if (manifest.robot.joint_upper_limits.size() != 18)
    {
        return ValidationResult::Error("robot.joint_upper_limits must have 18 entries", "robot.joint_upper_limits");
    }
    if (manifest.robot.joint_velocity_limits.size() != 18)
    {
        return ValidationResult::Error("robot.joint_velocity_limits must have 18 entries", "robot.joint_velocity_limits");
    }

    if (manifest.policy.mode != "dog_only")
    {
        return ValidationResult::Error("policy.mode must be dog_only", "policy.mode");
    }
    if (manifest.policy.action_dim != 12)
    {
        return ValidationResult::Error("policy.action_dim must be 12", "policy.action_dim");
    }
    if (manifest.policy.observation_dim != 260)
    {
        return ValidationResult::Error("policy.observation_dim must be 260", "policy.observation_dim");
    }
    if (manifest.policy.policy_rate_hz != 50)
    {
        return ValidationResult::Error("policy.policy_rate_hz must be 50", "policy.policy_rate_hz");
    }
    if (manifest.policy.model_path.empty())
    {
        return ValidationResult::Error("policy.model_path must be non-empty", "policy.model_path");
    }
    if (manifest.policy.enable_arm_tracking_error_obs)
    {
        return ValidationResult::Error("policy.enable_arm_tracking_error_obs must be false in first freeze", "policy.enable_arm_tracking_error_obs");
    }
    if (manifest.policy.enable_arm_command_delta_obs)
    {
        return ValidationResult::Error("policy.enable_arm_command_delta_obs must be false in first freeze", "policy.enable_arm_command_delta_obs");
    }

    if (manifest.body_adapter.interface != "unitree_sdk2")
    {
        return ValidationResult::Error("body_adapter.interface must be unitree_sdk2", "body_adapter.interface");
    }
    if (manifest.body_adapter.command_rate_hz != manifest.coordinator.rate_hz)
    {
        return ValidationResult::Error("body_adapter.command_rate_hz must match coordinator.rate_hz", "body_adapter.command_rate_hz");
    }
    if (!manifest.body_adapter.require_lowstate)
    {
        return ValidationResult::Error("body_adapter.require_lowstate must be true", "body_adapter.require_lowstate");
    }
    if (manifest.body_adapter.lowstate_timeout_ms != 50)
    {
        return ValidationResult::Error("body_adapter.lowstate_timeout_ms must be 50", "body_adapter.lowstate_timeout_ms");
    }

    if (manifest.arm_adapter.interface != "arx5_sdk")
    {
        return ValidationResult::Error("arm_adapter.interface must be arx5_sdk", "arm_adapter.interface");
    }
    if (manifest.arm_adapter.arm_cmd_topic != "/arx_x5/joint_cmd")
    {
        return ValidationResult::Error("arm_adapter.arm_cmd_topic must be /arx_x5/joint_cmd",
            "arm_adapter.arm_cmd_topic");
    }
    if (manifest.arm_adapter.arm_state_topic != "/arx_x5/joint_state")
    {
        return ValidationResult::Error("arm_adapter.arm_state_topic must be /arx_x5/joint_state",
            "arm_adapter.arm_state_topic");
    }
    if (manifest.arm_adapter.arm_joint_command_topic != "/arm_joint_pos_cmd")
    {
        return ValidationResult::Error("arm_adapter.arm_joint_command_topic must be /arm_joint_pos_cmd",
            "arm_adapter.arm_joint_command_topic");
    }
    if (manifest.arm_adapter.arm_target_rate_hz != 200)
    {
        return ValidationResult::Error("arm_adapter.arm_target_rate_hz must be 200", "arm_adapter.arm_target_rate_hz");
    }
    if (manifest.arm_adapter.servo_rate_hz < manifest.coordinator.rate_hz)
    {
        return ValidationResult::Error("arm_adapter.servo_rate_hz must be >= coordinator.rate_hz", "arm_adapter.servo_rate_hz");
    }
    if (!manifest.arm_adapter.background_send_recv)
    {
        return ValidationResult::Error("arm_adapter.background_send_recv must be true", "arm_adapter.background_send_recv");
    }
    if (!IsPositive(manifest.arm_adapter.controller_dt))
    {
        return ValidationResult::Error("arm_adapter.controller_dt must be positive", "arm_adapter.controller_dt");
    }
    if (!manifest.arm_adapter.require_live_state)
    {
        return ValidationResult::Error("arm_adapter.require_live_state must be true", "arm_adapter.require_live_state");
    }
    if (manifest.arm_adapter.arm_state_timeout_ms != 50)
    {
        return ValidationResult::Error("arm_adapter.arm_state_timeout_ms must be 50", "arm_adapter.arm_state_timeout_ms");
    }
    if (!IsPositive(manifest.arm_adapter.arm_tracking_error_limit))
    {
        return ValidationResult::Error("arm_adapter.arm_tracking_error_limit must be positive", "arm_adapter.arm_tracking_error_limit");
    }

    if (manifest.coordinator.rate_hz != 200)
    {
        return ValidationResult::Error("coordinator.rate_hz must be 200", "coordinator.rate_hz");
    }
    if (manifest.coordinator.body_command_expire_ms != 15)
    {
        return ValidationResult::Error("coordinator.body_command_expire_ms must be 15", "coordinator.body_command_expire_ms");
    }
    if (manifest.coordinator.arm_command_expire_ms != 15)
    {
        return ValidationResult::Error("coordinator.arm_command_expire_ms must be 15", "coordinator.arm_command_expire_ms");
    }
    if (!IsPositive(manifest.coordinator.zero_cmd_xy_drift_limit))
    {
        return ValidationResult::Error("coordinator.zero_cmd_xy_drift_limit must be positive", "coordinator.zero_cmd_xy_drift_limit");
    }
    if (!IsPositive(manifest.coordinator.zero_cmd_yaw_drift_limit))
    {
        return ValidationResult::Error("coordinator.zero_cmd_yaw_drift_limit must be positive", "coordinator.zero_cmd_yaw_drift_limit");
    }
    if (!manifest.coordinator.enable_drift_anchor)
    {
        return ValidationResult::Error("coordinator.enable_drift_anchor must be true", "coordinator.enable_drift_anchor");
    }

    if (!IsPositive(manifest.supervisor.probe_window_sec))
    {
        return ValidationResult::Error("supervisor.probe_window_sec must be positive", "supervisor.probe_window_sec");
    }
    if (!IsPositive(manifest.supervisor.degraded_timeout_sec))
    {
        return ValidationResult::Error("supervisor.degraded_timeout_sec must be positive", "supervisor.degraded_timeout_sec");
    }
    if (manifest.supervisor.policy_timeout_ms != 100)
    {
        return ValidationResult::Error("supervisor.policy_timeout_ms must be 100", "supervisor.policy_timeout_ms");
    }
    if (!manifest.supervisor.fault_latched_requires_manual_reset)
    {
        return ValidationResult::Error("supervisor.fault_latched_requires_manual_reset must be true", "supervisor.fault_latched_requires_manual_reset");
    }

    if (!manifest.ops.ros2_enabled)
    {
        return ValidationResult::Error("ops.ros2_enabled must be true", "ops.ros2_enabled");
    }
    if (!manifest.ops.ros2_mirror_only)
    {
        return ValidationResult::Error("ops.ros2_mirror_only must be true", "ops.ros2_mirror_only");
    }
    if (!manifest.ops.bag_enabled)
    {
        return ValidationResult::Error("ops.bag_enabled must be true", "ops.bag_enabled");
    }
    if (manifest.ops.bridge_rmw_implementation != "rmw_cyclonedds_cpp")
    {
        return ValidationResult::Error("ops.bridge_rmw_implementation must be rmw_cyclonedds_cpp",
            "ops.bridge_rmw_implementation");
    }
    if (manifest.ops.go2_rmw_implementation != "rmw_fastrtps_cpp")
    {
        return ValidationResult::Error("ops.go2_rmw_implementation must be rmw_fastrtps_cpp",
            "ops.go2_rmw_implementation");
    }
    if (manifest.ops.diagnostics_rate_hz != 50)
    {
        return ValidationResult::Error("ops.diagnostics_rate_hz must be 50", "ops.diagnostics_rate_hz");
    }

    return ValidationResult::Ok();
}

void DeployManifestLoader::RequireMap(const YAML::Node& node, const char* path, ValidationResult* result)
{
    if (result == nullptr || !result->is_valid)
    {
        return;
    }
    if (!node || !node.IsMap())
    {
        *result = ValidationResult::Error(std::string(path) + " must be a map", path);
    }
}

void DeployManifestLoader::RequireAllowedKeys(
    const YAML::Node& node,
    const std::vector<std::string>& allowed_keys,
    const char* path,
    ValidationResult* result)
{
    if (result == nullptr || !result->is_valid)
    {
        return;
    }
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        const std::string key = it->first.as<std::string>();
        if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end())
        {
            *result = ValidationResult::Error(std::string("unexpected key: ") + key, PathJoin(path, key.c_str()));
            return;
        }
    }
}

void DeployManifestLoader::RequireScalarString(
    const YAML::Node& node,
    const char* key,
    const char* path,
    std::string* out,
    ValidationResult* result)
{
    if (result != nullptr && !result->is_valid)
    {
        return;
    }
    if (!node[key] || !node[key].IsScalar())
    {
        if (result != nullptr)
        {
            *result = ValidationResult::Error(std::string(key) + " must be a scalar string", PathJoin(path, key));
        }
        return;
    }
    if (out != nullptr)
    {
        *out = node[key].as<std::string>();
    }
}

void DeployManifestLoader::RequireScalarInt(
    const YAML::Node& node,
    const char* key,
    const char* path,
    int* out,
    ValidationResult* result)
{
    if (result != nullptr && !result->is_valid)
    {
        return;
    }
    if (!node[key] || !node[key].IsScalar())
    {
        if (result != nullptr)
        {
            *result = ValidationResult::Error(std::string(key) + " must be a scalar integer", PathJoin(path, key));
        }
        return;
    }
    if (out != nullptr)
    {
        *out = node[key].as<int>();
    }
}

void DeployManifestLoader::RequireScalarDouble(
    const YAML::Node& node,
    const char* key,
    const char* path,
    double* out,
    ValidationResult* result)
{
    if (result != nullptr && !result->is_valid)
    {
        return;
    }
    if (!node[key] || !node[key].IsScalar())
    {
        if (result != nullptr)
        {
            *result = ValidationResult::Error(std::string(key) + " must be a scalar number", PathJoin(path, key));
        }
        return;
    }
    if (out != nullptr)
    {
        *out = node[key].as<double>();
    }
}

void DeployManifestLoader::RequireScalarBool(
    const YAML::Node& node,
    const char* key,
    const char* path,
    bool* out,
    ValidationResult* result)
{
    if (result != nullptr && !result->is_valid)
    {
        return;
    }
    if (!node[key] || !node[key].IsScalar())
    {
        if (result != nullptr)
        {
            *result = ValidationResult::Error(std::string(key) + " must be a scalar bool", PathJoin(path, key));
        }
        return;
    }
    if (out != nullptr)
    {
        *out = node[key].as<bool>();
    }
}

void DeployManifestLoader::RequireIntVector(
    const YAML::Node& node,
    const char* key,
    size_t expected_size,
    const char* path,
    std::vector<int>* out,
    ValidationResult* result)
{
    if (result != nullptr && !result->is_valid)
    {
        return;
    }
    if (!node[key] || !node[key].IsSequence())
    {
        if (result != nullptr)
        {
            *result = ValidationResult::Error(std::string(key) + " must be a sequence", PathJoin(path, key));
        }
        return;
    }
    if (node[key].size() != expected_size)
    {
        if (result != nullptr)
        {
            std::ostringstream oss;
            oss << key << " size mismatch: expected " << expected_size << ", got " << node[key].size();
            *result = ValidationResult::Error(oss.str(), PathJoin(path, key));
        }
        return;
    }
    if (out != nullptr)
    {
        out->clear();
        out->reserve(expected_size);
        for (const auto& item : node[key])
        {
            out->push_back(item.as<int>());
        }
    }
}

void DeployManifestLoader::RequireDoubleVector(
    const YAML::Node& node,
    const char* key,
    size_t expected_size,
    const char* path,
    std::vector<double>* out,
    ValidationResult* result)
{
    if (result != nullptr && !result->is_valid)
    {
        return;
    }
    if (!node[key] || !node[key].IsSequence())
    {
        if (result != nullptr)
        {
            *result = ValidationResult::Error(std::string(key) + " must be a sequence", PathJoin(path, key));
        }
        return;
    }
    if (node[key].size() != expected_size)
    {
        if (result != nullptr)
        {
            std::ostringstream oss;
            oss << key << " size mismatch: expected " << expected_size << ", got " << node[key].size();
            *result = ValidationResult::Error(oss.str(), PathJoin(path, key));
        }
        return;
    }
    if (out != nullptr)
    {
        out->clear();
        out->reserve(expected_size);
        for (const auto& item : node[key])
        {
            out->push_back(item.as<double>());
        }
    }
}

std::string DeployManifestLoader::SerializeManifest(const DeployManifest& manifest, bool include_generated_at)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed, std::ios::floatfield);
    oss << std::setprecision(6);

    oss << "meta.manifest_version=" << manifest.meta.manifest_version << '\n';
    oss << "meta.protocol_version=" << manifest.meta.protocol_version << '\n';
    oss << "meta.policy_id=" << manifest.meta.policy_id << '\n';
    if (include_generated_at)
    {
        oss << "meta.generated_at=" << manifest.meta.generated_at << '\n';
    }

    oss << "robot.name=" << manifest.robot.name << '\n';
    oss << "robot.leg_joint_count=" << manifest.robot.leg_joint_count << '\n';
    oss << "robot.arm_joint_count=" << manifest.robot.arm_joint_count << '\n';
    oss << "robot.gripper_enabled=" << (manifest.robot.gripper_enabled ? 1 : 0) << '\n';
    oss << "robot.joint_mapping=";
    for (size_t i = 0; i < manifest.robot.joint_mapping.size(); ++i)
    {
        if (i > 0) oss << ',';
        oss << manifest.robot.joint_mapping[i];
    }
    oss << '\n';

    auto append_double_vector = [&oss](const char* name, const std::vector<double>& values)
    {
        oss << name << '=';
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0) oss << ',';
            oss << values[i];
        }
        oss << '\n';
    };
    append_double_vector("robot.joint_lower_limits", manifest.robot.joint_lower_limits);
    append_double_vector("robot.joint_upper_limits", manifest.robot.joint_upper_limits);
    append_double_vector("robot.joint_velocity_limits", manifest.robot.joint_velocity_limits);

    oss << "policy.mode=" << manifest.policy.mode << '\n';
    oss << "policy.action_dim=" << manifest.policy.action_dim << '\n';
    oss << "policy.observation_dim=" << manifest.policy.observation_dim << '\n';
    oss << "policy.policy_rate_hz=" << manifest.policy.policy_rate_hz << '\n';
    oss << "policy.model_path=" << manifest.policy.model_path << '\n';
    oss << "policy.enable_arm_tracking_error_obs=" << (manifest.policy.enable_arm_tracking_error_obs ? 1 : 0) << '\n';
    oss << "policy.enable_arm_command_delta_obs=" << (manifest.policy.enable_arm_command_delta_obs ? 1 : 0) << '\n';

    oss << "body_adapter.interface=" << manifest.body_adapter.interface << '\n';
    oss << "body_adapter.network_interface=" << manifest.body_adapter.network_interface << '\n';
    oss << "body_adapter.command_rate_hz=" << manifest.body_adapter.command_rate_hz << '\n';
    oss << "body_adapter.require_lowstate=" << (manifest.body_adapter.require_lowstate ? 1 : 0) << '\n';
    oss << "body_adapter.lowstate_timeout_ms=" << manifest.body_adapter.lowstate_timeout_ms << '\n';

    oss << "arm_adapter.interface=" << manifest.arm_adapter.interface << '\n';
    oss << "arm_adapter.can_interface=" << manifest.arm_adapter.can_interface << '\n';
    oss << "arm_adapter.arm_cmd_topic=" << manifest.arm_adapter.arm_cmd_topic << '\n';
    oss << "arm_adapter.arm_state_topic=" << manifest.arm_adapter.arm_state_topic << '\n';
    oss << "arm_adapter.arm_joint_command_topic=" << manifest.arm_adapter.arm_joint_command_topic << '\n';
    oss << "arm_adapter.arm_target_rate_hz=" << manifest.arm_adapter.arm_target_rate_hz << '\n';
    oss << "arm_adapter.servo_rate_hz=" << manifest.arm_adapter.servo_rate_hz << '\n';
    oss << "arm_adapter.background_send_recv=" << (manifest.arm_adapter.background_send_recv ? 1 : 0) << '\n';
    oss << "arm_adapter.controller_dt=" << manifest.arm_adapter.controller_dt << '\n';
    oss << "arm_adapter.require_live_state=" << (manifest.arm_adapter.require_live_state ? 1 : 0) << '\n';
    oss << "arm_adapter.arm_state_timeout_ms=" << manifest.arm_adapter.arm_state_timeout_ms << '\n';
    oss << "arm_adapter.arm_tracking_error_limit=" << manifest.arm_adapter.arm_tracking_error_limit << '\n';

    oss << "coordinator.rate_hz=" << manifest.coordinator.rate_hz << '\n';
    oss << "coordinator.body_command_expire_ms=" << manifest.coordinator.body_command_expire_ms << '\n';
    oss << "coordinator.arm_command_expire_ms=" << manifest.coordinator.arm_command_expire_ms << '\n';
    oss << "coordinator.zero_cmd_xy_drift_limit=" << manifest.coordinator.zero_cmd_xy_drift_limit << '\n';
    oss << "coordinator.zero_cmd_yaw_drift_limit=" << manifest.coordinator.zero_cmd_yaw_drift_limit << '\n';
    oss << "coordinator.enable_drift_anchor=" << (manifest.coordinator.enable_drift_anchor ? 1 : 0) << '\n';

    oss << "supervisor.probe_window_sec=" << manifest.supervisor.probe_window_sec << '\n';
    oss << "supervisor.degraded_timeout_sec=" << manifest.supervisor.degraded_timeout_sec << '\n';
    oss << "supervisor.policy_timeout_ms=" << manifest.supervisor.policy_timeout_ms << '\n';
    oss << "supervisor.fault_latched_requires_manual_reset=" << (manifest.supervisor.fault_latched_requires_manual_reset ? 1 : 0) << '\n';

    oss << "ops.ros2_enabled=" << (manifest.ops.ros2_enabled ? 1 : 0) << '\n';
    oss << "ops.ros2_mirror_only=" << (manifest.ops.ros2_mirror_only ? 1 : 0) << '\n';
    oss << "ops.bridge_rmw_implementation=" << manifest.ops.bridge_rmw_implementation << '\n';
    oss << "ops.go2_rmw_implementation=" << manifest.ops.go2_rmw_implementation << '\n';
    oss << "ops.bag_enabled=" << (manifest.ops.bag_enabled ? 1 : 0) << '\n';
    oss << "ops.diagnostics_rate_hz=" << manifest.ops.diagnostics_rate_hz << '\n';
    return oss.str();
}

std::uint64_t DeployManifestLoader::Fnv1a64(const std::string& text)
{
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char c : text)
    {
        hash ^= static_cast<std::uint64_t>(c);
        hash *= 1099511628211ULL;
    }
    return hash;
}

bool DeployManifestLoader::IsPositive(double value)
{
    return value > 0.0;
}

} // namespace RLConfig
