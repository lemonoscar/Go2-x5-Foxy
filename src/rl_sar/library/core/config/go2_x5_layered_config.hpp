#ifndef GO2_X5_LAYERED_CONFIG_HPP
#define GO2_X5_LAYERED_CONFIG_HPP

#include <algorithm>
#include <initializer_list>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace RLConfig
{

enum class ConfigLayerDomain
{
    BaseRobot,
    PolicyRuntime,
    Deploy,
    Operator,
    Passthrough
};

struct ConfigLoadRecord
{
    std::string file_path;
    std::string file_name;
    std::string config_path;
};

struct BaseRobotConfig
{
    int num_of_dofs = 0;
    int arm_joint_start_index = 0;
    int arm_joint_count = 0;
    std::vector<float> default_dof_pos;
    std::vector<float> joint_lower_limits;
    std::vector<float> joint_upper_limits;
    std::vector<float> joint_velocity_limits;
    std::vector<int> joint_mapping;
};

struct PolicyRuntimeConfig
{
    std::string model_name;
    int num_observations = 0;
    std::vector<std::string> observations;
    std::vector<float> action_scale;
    std::vector<float> rl_kp;
    std::vector<float> rl_kd;
    int arm_command_size = 0;
};

struct DeployConfig
{
    std::string arm_control_mode;
    std::string arm_bridge_cmd_topic;
    std::string arm_bridge_state_topic;
    std::string arm_joint_command_topic;
    bool arm_bridge_require_state = false;
    bool arm_bridge_require_live_state = false;
    bool arm_bridge_shadow_feedback_enabled = true;
    bool arm_hold_enabled = false;
    float arm_bridge_state_timeout_sec = 0.0f;
    float arm_command_smoothing_time = 0.0f;
};

struct OperatorConfig
{
    bool real_deploy_exclusive_keyboard_control = false;
    float fixed_cmd_x = 0.0f;
    float fixed_cmd_y = 0.0f;
    float fixed_cmd_yaw = 0.0f;
};

struct Go2X5TypedConfig
{
    BaseRobotConfig base_robot;
    PolicyRuntimeConfig policy_runtime;
    DeployConfig deploy;
    OperatorConfig operator_config;
};

namespace detail
{

inline bool StartsWith(const std::string& value, const std::string& prefix)
{
    return value.rfind(prefix, 0) == 0;
}

inline bool IsOneOf(const std::string& key, std::initializer_list<const char*> values)
{
    for (const char* value : values)
    {
        if (key == value)
        {
            return true;
        }
    }
    return false;
}

inline std::string Join(const std::vector<std::string>& values, const char* separator)
{
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0)
        {
            oss << separator;
        }
        oss << values[i];
    }
    return oss.str();
}

template <typename T>
inline T ReadScalar(const YAML::Node& node, const char* key, const T& default_value)
{
    if (node[key])
    {
        return node[key].as<T>();
    }
    return default_value;
}

template <typename T>
inline std::vector<T> ReadVector(const YAML::Node& node, const char* key)
{
    if (node[key])
    {
        return node[key].as<std::vector<T>>();
    }
    return {};
}

inline ConfigLayerDomain ClassifyKey(const std::string& key)
{
    if (IsOneOf(key, {
            "dt", "decimation", "fixed_kp", "fixed_kd", "num_of_dofs", "wheel_indices",
            "torque_limits", "default_dof_pos", "joint_lower_limits", "joint_upper_limits",
            "joint_velocity_limits", "joint_effort_limits", "joint_kp_limits", "joint_kd_limits",
            "joint_names", "joint_controller_names", "joint_mapping", "arm_joint_start_index",
            "arm_joint_count", "arm_joint_lower_limits", "arm_joint_upper_limits", "init_base_height"}))
    {
        return ConfigLayerDomain::BaseRobot;
    }

    if (IsOneOf(key, {
            "model_name", "num_observations", "observations", "observations_history",
            "observations_history_priority", "clip_obs", "clip_actions_lower", "clip_actions_upper",
            "rl_kp", "rl_kd", "action_scale", "lin_vel_scale", "ang_vel_scale", "dof_pos_scale",
            "dof_vel_scale", "commands_scale", "arm_command_size"}))
    {
        return ConfigLayerDomain::PolicyRuntime;
    }

    if (IsOneOf(key, {
            "joystick_deadband", "arm_joint_command_topic", "arm_control_mode",
            "arm_bridge_cmd_topic", "arm_bridge_state_topic", "arm_bridge_require_state",
            "arm_bridge_require_live_state", "arm_bridge_shadow_feedback_enabled",
            "arm_bridge_state_timeout_sec", "arm_hold_enabled",
            "arm_lock", "arm_lock_pose", "arm_lock_base_on_pose", "arm_hold_pose",
            "arm_command_smoothing_time", "arm_output_filter_alpha", "arm_key_pose",
            "arm_sequence_interval", "arm_sequence_steps", "arm_sequence", "shutdown_soft_land_sec",
            "shutdown_hold_sec", "arm_shutdown_pose", "policy_inference_log_enabled"}))
    {
        return ConfigLayerDomain::Deploy;
    }

    if (IsOneOf(key, {
            "real_deploy_exclusive_keyboard_control", "fixed_cmd_x", "fixed_cmd_y", "fixed_cmd_yaw"}))
    {
        return ConfigLayerDomain::Operator;
    }

    return ConfigLayerDomain::Passthrough;
}

inline void MergeNodeInto(YAML::Node* destination, const YAML::Node& source)
{
    for (auto it = source.begin(); it != source.end(); ++it)
    {
        (*destination)[it->first.as<std::string>()] = it->second;
    }
}

inline void ValidateSequenceSize(
    const YAML::Node& node,
    const char* key,
    const size_t expected_size,
    std::vector<std::string>* errors)
{
    if (!node[key])
    {
        return;
    }
    if (!node[key].IsSequence())
    {
        errors->push_back(std::string(key) + " must be a sequence");
        return;
    }
    if (node[key].size() != expected_size)
    {
        std::ostringstream oss;
        oss << key << " size mismatch: expected " << expected_size << ", got " << node[key].size();
        errors->push_back(oss.str());
    }
}

} // namespace detail

inline bool IsGo2X5ConfigPath(const std::string& file_path)
{
    return file_path == "go2_x5" || detail::StartsWith(file_path, "go2_x5/");
}

class LayeredGo2X5ConfigState
{
public:
    void ApplyLoad(
        const std::string& file_path,
        const std::string& file_name,
        const std::string& config_path,
        const YAML::Node& scoped_config)
    {
        this->load_history_.push_back({file_path, file_name, config_path});

        if (scoped_config && scoped_config.IsMap())
        {
            for (auto it = scoped_config.begin(); it != scoped_config.end(); ++it)
            {
                const std::string key = it->first.as<std::string>();
                switch (detail::ClassifyKey(key))
                {
                case ConfigLayerDomain::BaseRobot:
                    this->base_robot_layer_[key] = it->second;
                    break;
                case ConfigLayerDomain::PolicyRuntime:
                    this->policy_runtime_layer_[key] = it->second;
                    break;
                case ConfigLayerDomain::Deploy:
                    this->deploy_layer_[key] = it->second;
                    break;
                case ConfigLayerDomain::Operator:
                    this->operator_layer_[key] = it->second;
                    break;
                case ConfigLayerDomain::Passthrough:
                    this->passthrough_layer_[key] = it->second;
                    break;
                }
            }
        }

        this->RebuildMergedNode();
        this->RebuildTypedConfig();
        this->Validate();
    }

    bool IsActive() const
    {
        return !this->load_history_.empty();
    }

    bool HasValidationErrors() const
    {
        return !this->validation_errors_.empty();
    }

    std::string ValidationSummary() const
    {
        std::ostringstream oss;
        oss << "go2_x5 layered config validation failed";
        if (!this->load_history_.empty())
        {
            oss << " after loading ";
            for (size_t i = 0; i < this->load_history_.size(); ++i)
            {
                if (i > 0)
                {
                    oss << " -> ";
                }
                oss << this->load_history_[i].file_path << "/" << this->load_history_[i].file_name;
            }
        }
        oss << ": " << detail::Join(this->validation_errors_, "; ");
        return oss.str();
    }

    const YAML::Node& MergedNode() const
    {
        return this->merged_node_;
    }

    const Go2X5TypedConfig& Typed() const
    {
        return this->typed_config_;
    }

    const std::vector<ConfigLoadRecord>& LoadHistory() const
    {
        return this->load_history_;
    }

private:
    bool HasPolicyRuntimeLoad() const
    {
        return std::any_of(
            this->load_history_.begin(),
            this->load_history_.end(),
            [] (const ConfigLoadRecord& record)
            {
                return record.file_name == "config.yaml" && detail::StartsWith(record.file_path, "go2_x5/");
            });
    }

    void RebuildMergedNode()
    {
        this->merged_node_ = YAML::Node(YAML::NodeType::Map);
        detail::MergeNodeInto(&this->merged_node_, this->base_robot_layer_);
        detail::MergeNodeInto(&this->merged_node_, this->policy_runtime_layer_);
        detail::MergeNodeInto(&this->merged_node_, this->deploy_layer_);
        detail::MergeNodeInto(&this->merged_node_, this->operator_layer_);
        detail::MergeNodeInto(&this->merged_node_, this->passthrough_layer_);
    }

    void RebuildTypedConfig()
    {
        const YAML::Node& node = this->merged_node_;
        this->typed_config_ = Go2X5TypedConfig{};

        this->typed_config_.base_robot.num_of_dofs = detail::ReadScalar<int>(node, "num_of_dofs", 0);
        this->typed_config_.base_robot.arm_joint_start_index = detail::ReadScalar<int>(node, "arm_joint_start_index", 0);
        this->typed_config_.base_robot.arm_joint_count = detail::ReadScalar<int>(node, "arm_joint_count", 0);
        this->typed_config_.base_robot.default_dof_pos = detail::ReadVector<float>(node, "default_dof_pos");
        this->typed_config_.base_robot.joint_lower_limits = detail::ReadVector<float>(node, "joint_lower_limits");
        this->typed_config_.base_robot.joint_upper_limits = detail::ReadVector<float>(node, "joint_upper_limits");
        this->typed_config_.base_robot.joint_velocity_limits = detail::ReadVector<float>(node, "joint_velocity_limits");
        this->typed_config_.base_robot.joint_mapping = detail::ReadVector<int>(node, "joint_mapping");

        this->typed_config_.policy_runtime.model_name = detail::ReadScalar<std::string>(node, "model_name", "");
        this->typed_config_.policy_runtime.num_observations = detail::ReadScalar<int>(node, "num_observations", 0);
        this->typed_config_.policy_runtime.observations = detail::ReadVector<std::string>(node, "observations");
        this->typed_config_.policy_runtime.action_scale = detail::ReadVector<float>(node, "action_scale");
        this->typed_config_.policy_runtime.rl_kp = detail::ReadVector<float>(node, "rl_kp");
        this->typed_config_.policy_runtime.rl_kd = detail::ReadVector<float>(node, "rl_kd");
        this->typed_config_.policy_runtime.arm_command_size = detail::ReadScalar<int>(node, "arm_command_size", 0);

        this->typed_config_.deploy.arm_control_mode = detail::ReadScalar<std::string>(node, "arm_control_mode", "");
        this->typed_config_.deploy.arm_bridge_cmd_topic = detail::ReadScalar<std::string>(node, "arm_bridge_cmd_topic", "");
        this->typed_config_.deploy.arm_bridge_state_topic = detail::ReadScalar<std::string>(node, "arm_bridge_state_topic", "");
        this->typed_config_.deploy.arm_joint_command_topic = detail::ReadScalar<std::string>(node, "arm_joint_command_topic", "");
        this->typed_config_.deploy.arm_bridge_require_state = detail::ReadScalar<bool>(node, "arm_bridge_require_state", false);
        this->typed_config_.deploy.arm_bridge_require_live_state = detail::ReadScalar<bool>(node, "arm_bridge_require_live_state", false);
        this->typed_config_.deploy.arm_bridge_shadow_feedback_enabled =
            detail::ReadScalar<bool>(node, "arm_bridge_shadow_feedback_enabled", true);
        this->typed_config_.deploy.arm_hold_enabled = detail::ReadScalar<bool>(node, "arm_hold_enabled", false);
        this->typed_config_.deploy.arm_bridge_state_timeout_sec = detail::ReadScalar<float>(node, "arm_bridge_state_timeout_sec", 0.0f);
        this->typed_config_.deploy.arm_command_smoothing_time = detail::ReadScalar<float>(node, "arm_command_smoothing_time", 0.0f);

        this->typed_config_.operator_config.real_deploy_exclusive_keyboard_control =
            detail::ReadScalar<bool>(node, "real_deploy_exclusive_keyboard_control", false);
        this->typed_config_.operator_config.fixed_cmd_x = detail::ReadScalar<float>(node, "fixed_cmd_x", 0.0f);
        this->typed_config_.operator_config.fixed_cmd_y = detail::ReadScalar<float>(node, "fixed_cmd_y", 0.0f);
        this->typed_config_.operator_config.fixed_cmd_yaw = detail::ReadScalar<float>(node, "fixed_cmd_yaw", 0.0f);
    }

    void Validate()
    {
        this->validation_errors_.clear();

        const int num_of_dofs = this->typed_config_.base_robot.num_of_dofs;
        if (num_of_dofs <= 0)
        {
            this->validation_errors_.push_back("num_of_dofs must be greater than zero");
            return;
        }

        detail::ValidateSequenceSize(this->merged_node_, "default_dof_pos", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
        detail::ValidateSequenceSize(this->merged_node_, "joint_lower_limits", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
        detail::ValidateSequenceSize(this->merged_node_, "joint_upper_limits", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
        detail::ValidateSequenceSize(this->merged_node_, "joint_velocity_limits", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
        detail::ValidateSequenceSize(this->merged_node_, "joint_mapping", static_cast<size_t>(num_of_dofs), &this->validation_errors_);

        const int arm_joint_start_index = this->typed_config_.base_robot.arm_joint_start_index;
        const int arm_joint_count = this->typed_config_.base_robot.arm_joint_count;
        if (arm_joint_start_index < 0)
        {
            this->validation_errors_.push_back("arm_joint_start_index must be non-negative");
        }
        if (arm_joint_count < 0)
        {
            this->validation_errors_.push_back("arm_joint_count must be non-negative");
        }
        if (arm_joint_start_index + arm_joint_count > num_of_dofs)
        {
            this->validation_errors_.push_back("arm_joint_start_index + arm_joint_count exceeds num_of_dofs");
        }

        const int arm_command_size = this->typed_config_.policy_runtime.arm_command_size;
        if (arm_command_size > 0 && arm_joint_count > 0 && arm_command_size != arm_joint_count)
        {
            this->validation_errors_.push_back("arm_command_size must match arm_joint_count");
        }

        const size_t expected_arm_size = static_cast<size_t>(std::max(0, arm_joint_count));
        if (expected_arm_size > 0)
        {
            detail::ValidateSequenceSize(this->merged_node_, "arm_hold_pose", expected_arm_size, &this->validation_errors_);
            detail::ValidateSequenceSize(this->merged_node_, "arm_key_pose", expected_arm_size, &this->validation_errors_);
            detail::ValidateSequenceSize(this->merged_node_, "arm_shutdown_pose", expected_arm_size, &this->validation_errors_);
        }

        if (!this->HasPolicyRuntimeLoad())
        {
            return;
        }

        if (this->typed_config_.policy_runtime.model_name.empty())
        {
            this->validation_errors_.push_back("model_name must be present once policy runtime config is loaded");
        }
        if (this->typed_config_.policy_runtime.num_observations <= 0)
        {
            this->validation_errors_.push_back("num_observations must be present once policy runtime config is loaded");
        }
        if (this->typed_config_.policy_runtime.observations.empty())
        {
            this->validation_errors_.push_back("observations must be present once policy runtime config is loaded");
        }

        detail::ValidateSequenceSize(this->merged_node_, "action_scale", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
        detail::ValidateSequenceSize(this->merged_node_, "rl_kp", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
        detail::ValidateSequenceSize(this->merged_node_, "rl_kd", static_cast<size_t>(num_of_dofs), &this->validation_errors_);
    }

    YAML::Node base_robot_layer_ = YAML::Node(YAML::NodeType::Map);
    YAML::Node policy_runtime_layer_ = YAML::Node(YAML::NodeType::Map);
    YAML::Node deploy_layer_ = YAML::Node(YAML::NodeType::Map);
    YAML::Node operator_layer_ = YAML::Node(YAML::NodeType::Map);
    YAML::Node passthrough_layer_ = YAML::Node(YAML::NodeType::Map);
    YAML::Node merged_node_ = YAML::Node(YAML::NodeType::Map);
    Go2X5TypedConfig typed_config_;
    std::vector<ConfigLoadRecord> load_history_;
    std::vector<std::string> validation_errors_;
};

} // namespace RLConfig

#endif // GO2_X5_LAYERED_CONFIG_HPP
