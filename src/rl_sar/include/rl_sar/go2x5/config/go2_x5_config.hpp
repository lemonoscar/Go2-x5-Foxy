
#ifndef GO2_X5_CONFIG_HPP
#define GO2_X5_CONFIG_HPP

#include <algorithm>
#include <cctype>
#include "library/core/config/config_loader.hpp"
#include <string>
#include <vector>

namespace Go2X5Config {

/**
 * @brief Go2-X5 specific configuration wrapper
 *
 * Provides type-safe access to all Go2-X5 configuration values
 * using the ConfigLoader module.
 */
class Go2X5Config {
public:
    explicit Go2X5Config(RLConfig::ConfigLoader& loader)
        : loader_(loader) {}

    // ========================================================================
    // Basic Robot Configuration
    // ========================================================================

    int GetNumDofs() const {
        return loader_.Get<int>("num_of_dofs", 18);
    }

    float GetDt() const {
        return loader_.Get<float>("dt", 0.02f);
    }

    int GetDecimation() const {
        return loader_.Get<int>("decimation", 1);
    }

    std::vector<float> GetDefaultDofPos() const {
        return loader_.Get<std::vector<float>>("default_dof_pos", {});
    }

    std::vector<int> GetJointMapping() const {
        return loader_.Get<std::vector<int>>("joint_mapping", {});
    }

    // ========================================================================
    // Control Gains
    // ========================================================================

    std::vector<float> GetFixedKp() const {
        return loader_.Get<std::vector<float>>("fixed_kp", {});
    }

    std::vector<float> GetFixedKd() const {
        return loader_.Get<std::vector<float>>("fixed_kd", {});
    }

    std::vector<float> GetRlKp() const {
        return loader_.Get<std::vector<float>>("rl_kp", {});
    }

    std::vector<float> GetRlKd() const {
        return loader_.Get<std::vector<float>>("rl_kd", {});
    }

    std::vector<float> GetActionScale() const {
        return loader_.Get<std::vector<float>>("action_scale", {});
    }

    // ========================================================================
    // Arm Configuration
    // ========================================================================

    int GetArmJointCount() const {
        return loader_.Get<int>("arm_joint_count", 6);
    }

    int GetArmJointStartIndex() const {
        return loader_.Get<int>("arm_joint_start_index", 12);
    }

    int GetArmCommandSize() const {
        return loader_.Get<int>("arm_command_size", 0);
    }

    std::string GetArmControlMode() const {
        std::string mode = loader_.Get<std::string>("arm_control_mode", "unitree");
        // Normalize to lowercase
        std::transform(mode.begin(), mode.end(), mode.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return mode;
    }

    bool IsArmSplitControlEnabled() const {
        const std::string mode = GetArmControlMode();
        return (mode == "split" || mode == "arx_x5" || mode == "bridge");
    }

    bool GetArmHoldEnabled() const {
        return loader_.Get<bool>("arm_hold_enabled", true);
    }

    std::vector<float> GetArmHoldPose() const {
        return loader_.Get<std::vector<float>>("arm_hold_pose", {});
    }

    std::vector<float> GetArmKeyPose() const {
        return loader_.Get<std::vector<float>>("arm_key_pose", {});
    }

    std::vector<float> GetArmLockPose() const {
        return loader_.Get<std::vector<float>>("arm_lock_pose", {});
    }

    std::vector<float> GetArmShutdownPose() const {
        return loader_.Get<std::vector<float>>("arm_shutdown_pose", {});
    }

    float GetArmCommandSmoothingTime() const {
        return loader_.Get<float>("arm_command_smoothing_time", 0.2f);
    }

    float GetArmOutputFilterAlpha() const {
        return loader_.Get<float>("arm_output_filter_alpha", 0.0f);
    }

    bool GetArmLock() const {
        return loader_.Get<bool>("arm_lock", false);
    }

    // ========================================================================
    // Arm Bridge Configuration
    // ========================================================================

    std::string GetArmBridgeCmdTopic() const {
        return loader_.Get<std::string>("arm_bridge_cmd_topic", "/arx_x5/joint_cmd");
    }

    std::string GetArmBridgeStateTopic() const {
        return loader_.Get<std::string>("arm_bridge_state_topic", "/arx_x5/joint_state");
    }

    std::string GetArmJointCommandTopic() const {
        return loader_.Get<std::string>("arm_joint_command_topic", "/arm_joint_pos_cmd");
    }

    bool GetArmBridgeRequireState() const {
        return loader_.Get<bool>("arm_bridge_require_state", IsArmSplitControlEnabled());
    }

    bool GetArmBridgeRequireLiveState() const {
        return loader_.Get<bool>("arm_bridge_require_live_state", GetArmBridgeRequireState());
    }

    bool GetArmBridgeShadowFeedbackEnabled() const {
        return loader_.Get<bool>("arm_bridge_shadow_feedback_enabled", true);
    }

    float GetArmBridgeStateTimeoutSec() const {
        return loader_.Get<float>("arm_bridge_state_timeout_sec", 0.25f);
    }

    // ========================================================================
    // Safety Limits
    // ========================================================================

    std::vector<float> GetJointLowerLimits() const {
        return loader_.Get<std::vector<float>>("joint_lower_limits", {});
    }

    std::vector<float> GetJointUpperLimits() const {
        return loader_.Get<std::vector<float>>("joint_upper_limits", {});
    }

    std::vector<float> GetJointVelocityLimits() const {
        return loader_.Get<std::vector<float>>("joint_velocity_limits", {});
    }

    std::vector<float> GetJointEffortLimits() const {
        return loader_.Get<std::vector<float>>("joint_effort_limits", {});
    }

    std::vector<float> GetJointKpLimits() const {
        return loader_.Get<std::vector<float>>("joint_kp_limits", {});
    }

    std::vector<float> GetJointKdLimits() const {
        return loader_.Get<std::vector<float>>("joint_kd_limits", {});
    }

    std::vector<float> GetTorqueLimits() const {
        return loader_.Get<std::vector<float>>("torque_limits", {});
    }

    std::vector<std::string> GetJointNames() const {
        return loader_.Get<std::vector<std::string>>("joint_names", {});
    }

    std::vector<float> GetArmJointLowerLimits() const {
        return loader_.Get<std::vector<float>>("arm_joint_lower_limits", {});
    }

    std::vector<float> GetArmJointUpperLimits() const {
        return loader_.Get<std::vector<float>>("arm_joint_upper_limits", {});
    }

    // ========================================================================
    // Operator Control Configuration
    // ========================================================================

    bool GetRealDeployExclusiveKeyboardControl() const {
        return loader_.Get<bool>("real_deploy_exclusive_keyboard_control", true);
    }

    float GetCmdVelAlpha() const {
        return loader_.Get<float>("cmd_vel_alpha", 0.2f);
    }

    float GetJoystickDeadband() const {
        return loader_.Get<float>("joystick_deadband", 0.05f);
    }

    bool GetKey2PreferTopicCommand() const {
        return loader_.Get<bool>("key2_prefer_topic_command", true);
    }

    bool GetKey1PreferNavigationMode() const {
        return loader_.Get<bool>("key1_prefer_navigation_mode", false);
    }

    bool GetKey1PublishCmdVelOnNavigation() const {
        return loader_.Get<bool>("key1_publish_cmd_vel_on_navigation", true);
    }

    float GetKey1NavigationCmdX() const {
        return loader_.Get<float>("key1_navigation_cmd_x", 0.5f);
    }

    float GetKey1NavigationCmdY() const {
        return loader_.Get<float>("key1_navigation_cmd_y", 0.0f);
    }

    float GetKey1NavigationCmdYaw() const {
        return loader_.Get<float>("key1_navigation_cmd_yaw", 0.0f);
    }

    float GetFixedCmdX() const {
        return loader_.Get<float>("fixed_cmd_x", 0.0f);
    }

    float GetFixedCmdY() const {
        return loader_.Get<float>("fixed_cmd_y", 0.0f);
    }

    float GetFixedCmdYaw() const {
        return loader_.Get<float>("fixed_cmd_yaw", 0.0f);
    }

    // ========================================================================
    // Shutdown Configuration
    // ========================================================================

    float GetShutdownSoftLandSec() const {
        return loader_.Get<float>("shutdown_soft_land_sec", 2.0f);
    }

    float GetShutdownHoldSec() const {
        return loader_.Get<float>("shutdown_hold_sec", 0.6f);
    }

    std::vector<float> GetShutdownLiePose() const {
        return loader_.Get<std::vector<float>>("shutdown_lie_pose", {});
    }

    // ========================================================================
    // Logging Configuration
    // ========================================================================

    bool GetPolicyInferenceLogEnabled() const {
        return loader_.Get<bool>("policy_inference_log_enabled", true);
    }

    // ========================================================================
    // Policy Configuration
    // ========================================================================

    std::string GetModelName() const {
        return loader_.Get<std::string>("model_name", "");
    }

    int GetNumObservations() const {
        return loader_.Get<int>("num_observations", 0);
    }

    std::vector<std::string> GetObservations() const {
        return loader_.Get<std::vector<std::string>>("observations", {});
    }

    std::vector<int> GetObservationsHistory() const {
        return loader_.Get<std::vector<int>>("observations_history", {});
    }

    std::vector<float> GetClipActionsLower() const {
        return loader_.Get<std::vector<float>>("clip_actions_lower", {});
    }

    std::vector<float> GetClipActionsUpper() const {
        return loader_.Get<std::vector<float>>("clip_actions_upper", {});
    }

    // ========================================================================
    // Direct ConfigLoader Access (for advanced usage)
    // ========================================================================

    RLConfig::ConfigLoader& GetLoader() { return loader_; }
    const RLConfig::ConfigLoader& GetLoader() const { return loader_; }

private:
    RLConfig::ConfigLoader& loader_;
};

} // namespace Go2X5Config

#endif // GO2_X5_CONFIG_HPP
