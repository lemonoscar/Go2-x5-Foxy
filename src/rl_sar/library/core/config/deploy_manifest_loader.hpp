#ifndef RL_SAR_DEPLOY_MANIFEST_LOADER_HPP
#define RL_SAR_DEPLOY_MANIFEST_LOADER_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace RLConfig
{

struct DeployManifestValidationResult
{
    bool is_valid = true;
    std::string error_message;
    std::string error_path;

    DeployManifestValidationResult() = default;
    DeployManifestValidationResult(bool valid, const std::string& msg = "", const std::string& path = "")
        : is_valid(valid), error_message(msg), error_path(path)
    {
    }

    static DeployManifestValidationResult Ok()
    {
        return DeployManifestValidationResult(true);
    }

    static DeployManifestValidationResult Error(const std::string& msg, const std::string& path = "")
    {
        return DeployManifestValidationResult(false, msg, path);
    }
};

struct DeployManifestMeta
{
    int manifest_version = 1;
    int protocol_version = 1;
    std::string policy_id;
    std::string generated_at;
};

struct DeployManifestRobot
{
    std::string name;
    int leg_joint_count = 0;
    int arm_joint_count = 0;
    bool gripper_enabled = false;
    std::vector<int> joint_mapping;
    std::vector<double> joint_lower_limits;
    std::vector<double> joint_upper_limits;
    std::vector<double> joint_velocity_limits;
};

struct DeployManifestPolicy
{
    std::string mode;
    int action_dim = 0;
    int observation_dim = 0;
    int policy_rate_hz = 0;
    std::string model_path;
    bool enable_arm_tracking_error_obs = false;
    bool enable_arm_command_delta_obs = false;
};

struct DeployManifestVelocityEstimator
{
    double hip_length = 0.08;
    double thigh_length = 0.213;
    double calf_length = 0.213;
    double accelerometer_variance = 0.1;
    double sensor_variance = 0.1;
    double initial_variance = 0.1;
    int moving_window_size = 120;
    double foot_contact_force_threshold = 20.0;
};

struct DeployManifestBodyAdapter
{
    std::string interface;
    std::string network_interface;
    int command_rate_hz = 0;
    bool require_lowstate = false;
    int lowstate_timeout_ms = 0;
    bool enable_velocity_estimation = true;
    DeployManifestVelocityEstimator velocity_estimator;
};

struct DeployManifestArmAdapter
{
    std::string interface;
    std::string can_interface;
    std::string arm_cmd_topic;
    std::string arm_state_topic;
    std::string arm_joint_command_topic;
    int arm_target_rate_hz = 0;
    int servo_rate_hz = 0;
    bool background_send_recv = false;
    double controller_dt = 0.0;
    bool require_live_state = false;
    int arm_state_timeout_ms = 0;
    double arm_tracking_error_limit = 0.0;
};

struct DeployManifestCoordinator
{
    int rate_hz = 0;
    int body_command_expire_ms = 0;
    int arm_command_expire_ms = 0;
    double zero_cmd_xy_drift_limit = 0.0;
    double zero_cmd_yaw_drift_limit = 0.0;
    bool enable_drift_anchor = false;
};

struct DeployManifestSupervisor
{
    double probe_window_sec = 0.0;
    double degraded_timeout_sec = 0.0;
    int policy_timeout_ms = 0;
    bool fault_latched_requires_manual_reset = false;
};

struct DeployManifestOps
{
    bool ros2_enabled = false;
    bool ros2_mirror_only = false;
    std::string bridge_rmw_implementation;
    std::string go2_rmw_implementation;
    bool bag_enabled = false;
    int diagnostics_rate_hz = 0;
};

struct DeployManifest
{
    DeployManifestMeta meta;
    DeployManifestRobot robot;
    DeployManifestPolicy policy;
    DeployManifestBodyAdapter body_adapter;
    DeployManifestArmAdapter arm_adapter;
    DeployManifestCoordinator coordinator;
    DeployManifestSupervisor supervisor;
    DeployManifestOps ops;
};

class DeployManifestLoader
{
public:
    static constexpr const char* kDefaultManifestPath = "deploy/go2_x5_real.yaml";

    explicit DeployManifestLoader(std::string manifest_path = kDefaultManifestPath);

    DeployManifestValidationResult LoadDefaultManifest();
    DeployManifestValidationResult LoadFromFile(const std::string& manifest_path);
    DeployManifestValidationResult LoadFromNode(const YAML::Node& node);

    bool HasManifest() const;
    const std::string& ManifestPath() const;
    const DeployManifest& Manifest() const;
    DeployManifestValidationResult Validate() const;
    std::string ManifestDump() const;
    std::string ManifestHash() const;

private:
    static DeployManifestValidationResult ValidateNode(const YAML::Node& node);
    static DeployManifestValidationResult ValidateManifest(const DeployManifest& manifest);
    static DeployManifest ParseManifest(const YAML::Node& node);
    static void RequireMap(const YAML::Node& node, const char* path, DeployManifestValidationResult* result);
    static void RequireAllowedKeys(
        const YAML::Node& node,
        const std::vector<std::string>& allowed_keys,
        const char* path,
        DeployManifestValidationResult* result);
    static void RequireScalarString(
        const YAML::Node& node,
        const char* key,
        const char* path,
        std::string* out,
        DeployManifestValidationResult* result);
    static void RequireScalarInt(
        const YAML::Node& node,
        const char* key,
        const char* path,
        int* out,
        DeployManifestValidationResult* result);
    static void RequireScalarDouble(
        const YAML::Node& node,
        const char* key,
        const char* path,
        double* out,
        DeployManifestValidationResult* result);
    static void RequireScalarBool(
        const YAML::Node& node,
        const char* key,
        const char* path,
        bool* out,
        DeployManifestValidationResult* result);
    static void RequireIntVector(
        const YAML::Node& node,
        const char* key,
        size_t expected_size,
        const char* path,
        std::vector<int>* out,
        DeployManifestValidationResult* result);
    static void RequireDoubleVector(
        const YAML::Node& node,
        const char* key,
        size_t expected_size,
        const char* path,
        std::vector<double>* out,
        DeployManifestValidationResult* result);
    static std::string SerializeManifest(const DeployManifest& manifest, bool include_generated_at);
    static std::uint64_t Fnv1a64(const std::string& text);
    static bool IsPositive(double value);

    std::string manifest_path_;
    bool loaded_ = false;
    DeployManifest manifest_;
};

} // namespace RLConfig

#endif // RL_SAR_DEPLOY_MANIFEST_LOADER_HPP
