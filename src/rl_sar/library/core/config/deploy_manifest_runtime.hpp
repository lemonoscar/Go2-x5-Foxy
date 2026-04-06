#ifndef RL_SAR_DEPLOY_MANIFEST_RUNTIME_HPP
#define RL_SAR_DEPLOY_MANIFEST_RUNTIME_HPP

#include <cstdint>
#include <string>

#include "deploy_manifest_loader.hpp"

namespace RLConfig
{

struct DeployManifestRuntimeSnapshot
{
    std::string manifest_path;
    std::string manifest_hash;
    std::string policy_id;
    std::uint64_t policy_id_hash = 0;
    int protocol_version = 0;
    bool manifest_valid = false;
    bool ros2_enabled = false;
    bool ros2_mirror_only = false;
    bool bag_enabled = false;
    int diagnostics_rate_hz = 0;
    int policy_rate_hz = 0;
    int coordinator_rate_hz = 0;
    int body_command_rate_hz = 0;
    std::string body_interface;
    std::string body_network_interface;
    bool body_require_lowstate = false;
    int body_lowstate_timeout_ms = 0;
    int arm_target_rate_hz = 0;
    int arm_servo_rate_hz = 0;
    std::string arm_interface;
    std::string arm_can_interface;
    std::string arm_cmd_topic;
    std::string arm_state_topic;
    std::string arm_joint_command_topic;
    bool arm_background_send_recv = false;
    bool arm_require_live_state = false;
    int arm_state_timeout_ms = 0;
    double arm_tracking_error_limit = 0.0;
    int lowstate_timeout_ms = 0;
    double arm_controller_dt = 0.0;
    double probe_window_sec = 0.0;
    double degraded_timeout_sec = 0.0;
    int policy_timeout_ms = 0;
    int body_command_expire_ms = 0;
    int arm_command_expire_ms = 0;
    double zero_cmd_xy_drift_limit = 0.0;
    double zero_cmd_yaw_drift_limit = 0.0;
    bool enable_drift_anchor = false;
    std::string arm_bridge_transport;
    std::string bridge_rmw_implementation;
    std::string go2_rmw_implementation;
};

class DeployManifestRuntime
{
public:
    static constexpr const char* kDefaultManifestPath = DeployManifestLoader::kDefaultManifestPath;

    explicit DeployManifestRuntime(std::string manifest_path = kDefaultManifestPath);

    DeployManifestValidationResult Load();
    DeployManifestValidationResult LoadFromFile(const std::string& manifest_path);
    DeployManifestValidationResult LoadFromNode(const YAML::Node& node);

    bool HasManifest() const;
    const std::string& ManifestPath() const;
    const DeployManifest& Manifest() const;
    const DeployManifestLoader& Loader() const;
    DeployManifestValidationResult Validate() const;

    std::string ManifestHash() const;
    std::uint64_t PolicyIdHash() const;
    DeployManifestRuntimeSnapshot Snapshot() const;

    static std::string NormalizeManifestPath(const std::string& manifest_path);

private:
    static std::uint64_t Fnv1a64(const std::string& text);

    DeployManifestLoader loader_;
};

} // namespace RLConfig

#endif // RL_SAR_DEPLOY_MANIFEST_RUNTIME_HPP
