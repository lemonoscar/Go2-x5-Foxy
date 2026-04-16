#include "library/core/config/deploy_manifest_runtime.hpp"

#include <utility>

namespace RLConfig
{

using ValidationResult = DeployManifestValidationResult;

DeployManifestRuntime::DeployManifestRuntime(std::string manifest_path)
    : loader_(NormalizeManifestPath(manifest_path))
{
}

ValidationResult DeployManifestRuntime::Load()
{
    return this->loader_.LoadDefaultManifest();
}

ValidationResult DeployManifestRuntime::LoadFromFile(const std::string& manifest_path)
{
    return this->loader_.LoadFromFile(NormalizeManifestPath(manifest_path));
}

ValidationResult DeployManifestRuntime::LoadFromNode(const YAML::Node& node)
{
    return this->loader_.LoadFromNode(node);
}

bool DeployManifestRuntime::HasManifest() const
{
    return this->loader_.HasManifest();
}

const std::string& DeployManifestRuntime::ManifestPath() const
{
    return this->loader_.ManifestPath();
}

const DeployManifest& DeployManifestRuntime::Manifest() const
{
    return this->loader_.Manifest();
}

const DeployManifestLoader& DeployManifestRuntime::Loader() const
{
    return this->loader_;
}

ValidationResult DeployManifestRuntime::Validate() const
{
    return this->loader_.Validate();
}

std::string DeployManifestRuntime::ManifestHash() const
{
    return this->loader_.ManifestHash();
}

std::uint64_t DeployManifestRuntime::Fnv1a64(const std::string& text)
{
    std::uint64_t hash = 1469598103934665603ULL;
    for (unsigned char ch : text)
    {
        hash ^= static_cast<std::uint64_t>(ch);
        hash *= 1099511628211ULL;
    }
    return hash;
}

std::uint64_t DeployManifestRuntime::PolicyIdHash() const
{
    if (!this->loader_.HasManifest())
    {
        return 0ULL;
    }
    return Fnv1a64(this->loader_.Manifest().meta.policy_id);
}

DeployManifestRuntimeSnapshot DeployManifestRuntime::Snapshot() const
{
    DeployManifestRuntimeSnapshot snapshot;
    if (!this->loader_.HasManifest())
    {
        return snapshot;
    }

    const auto& manifest = this->loader_.Manifest();
    snapshot.manifest_path = this->loader_.ManifestPath();
    snapshot.manifest_hash = this->loader_.ManifestHash();
    snapshot.policy_id = manifest.meta.policy_id;
    snapshot.policy_id_hash = Fnv1a64(manifest.meta.policy_id);
    snapshot.protocol_version = manifest.meta.protocol_version;
    snapshot.manifest_valid = this->loader_.Validate().is_valid;
    snapshot.ros2_enabled = manifest.ops.ros2_enabled;
    snapshot.ros2_mirror_only = manifest.ops.ros2_mirror_only;
    snapshot.bag_enabled = manifest.ops.bag_enabled;
    snapshot.diagnostics_rate_hz = manifest.ops.diagnostics_rate_hz;
    snapshot.policy_rate_hz = manifest.policy.policy_rate_hz;
    snapshot.coordinator_rate_hz = manifest.coordinator.rate_hz;
    snapshot.body_command_rate_hz = manifest.body_adapter.command_rate_hz;
    snapshot.body_interface = manifest.body_adapter.interface;
    snapshot.body_network_interface = manifest.body_adapter.network_interface;
    snapshot.body_require_lowstate = manifest.body_adapter.require_lowstate;
    snapshot.body_lowstate_timeout_ms = manifest.body_adapter.lowstate_timeout_ms;
    snapshot.arm_target_rate_hz = manifest.arm_adapter.arm_target_rate_hz;
    snapshot.arm_servo_rate_hz = manifest.arm_adapter.servo_rate_hz;
    snapshot.arm_interface = manifest.arm_adapter.interface;
    snapshot.arm_can_interface = manifest.arm_adapter.can_interface;
    snapshot.arm_cmd_topic = manifest.arm_adapter.arm_cmd_topic;
    snapshot.arm_state_topic = manifest.arm_adapter.arm_state_topic;
    snapshot.arm_joint_command_topic = manifest.arm_adapter.arm_joint_command_topic;
    snapshot.arm_background_send_recv = manifest.arm_adapter.background_send_recv;
    snapshot.arm_require_live_state = manifest.arm_adapter.require_live_state;
    snapshot.arm_state_timeout_ms = manifest.arm_adapter.arm_state_timeout_ms;
    snapshot.arm_tracking_error_limit = manifest.arm_adapter.arm_tracking_error_limit;
    snapshot.lowstate_timeout_ms = manifest.body_adapter.lowstate_timeout_ms;
    snapshot.arm_controller_dt = manifest.arm_adapter.controller_dt;
    snapshot.probe_window_sec = manifest.supervisor.probe_window_sec;
    snapshot.degraded_timeout_sec = manifest.supervisor.degraded_timeout_sec;
    snapshot.policy_timeout_ms = manifest.supervisor.policy_timeout_ms;
    snapshot.body_command_expire_ms = manifest.coordinator.body_command_expire_ms;
    snapshot.arm_command_expire_ms = manifest.coordinator.arm_command_expire_ms;
    snapshot.zero_cmd_xy_drift_limit = manifest.coordinator.zero_cmd_xy_drift_limit;
    snapshot.zero_cmd_yaw_drift_limit = manifest.coordinator.zero_cmd_yaw_drift_limit;
    snapshot.enable_drift_anchor = manifest.coordinator.enable_drift_anchor;
    snapshot.arm_bridge_transport = manifest.ops.ros2_mirror_only ? "ipc" : "ros";
    snapshot.bridge_rmw_implementation = manifest.ops.bridge_rmw_implementation;
    snapshot.go2_rmw_implementation = manifest.ops.go2_rmw_implementation;
    snapshot.startup_sequence_enabled = manifest.startup_sequence.enabled;
    snapshot.startup_get_up_delay_sec = manifest.startup_sequence.get_up_delay_sec;
    snapshot.startup_rl_delay_after_get_up_sec = manifest.startup_sequence.rl_delay_after_get_up_sec;
    return snapshot;
}

std::string DeployManifestRuntime::NormalizeManifestPath(const std::string& manifest_path)
{
    if (manifest_path.empty())
    {
        return kDefaultManifestPath;
    }
    return manifest_path;
}

} // namespace RLConfig
