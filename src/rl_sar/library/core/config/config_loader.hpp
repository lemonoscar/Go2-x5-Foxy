#ifndef RL_SAR_CONFIG_LOADER_HPP
#define RL_SAR_CONFIG_LOADER_HPP

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace RLConfig {

/**
 * @brief Configuration layer enumeration
 *
 * Defines the priority order for configuration merging.
 * Lower values indicate higher priority (later loaded configs override earlier ones).
 */
enum class ConfigLayer {
    RobotDefault = 0,    // Hardcoded default values (lowest priority)
    BaseYaml = 1,        // base.yaml (robot-specific base config)
    RuntimeYaml = 2,     // robot_lab/config.yaml (runtime overrides)
    LaunchArgs = 3,      // Launch file arguments
    RuntimeOverride = 4  // Runtime programmatic overrides (highest priority)
};

/**
 * @brief Validation result structure
 */
struct ValidationResult {
    bool is_valid = true;
    std::string error_message;
    std::string error_path;  // Path to the invalid config item (e.g., "arm_control.kp")

    ValidationResult() = default;
    ValidationResult(bool valid, const std::string& msg = "", const std::string& path = "")
        : is_valid(valid), error_message(msg), error_path(path) {}

    static ValidationResult Ok() { return ValidationResult(true); }
    static ValidationResult Error(const std::string& msg, const std::string& path = "") {
        return ValidationResult(false, msg, path);
    }
};

/**
 * @brief Configuration descriptor for schema definition
 */
struct ConfigDescriptor {
    std::string key;
    std::string description;
    bool is_required;
    std::function<ValidationResult(const YAML::Node&)> validator;

    ConfigDescriptor(const std::string& k, bool required = false,
                    const std::string& desc = "")
        : key(k), is_required(required), description(desc) {}
};

/**
 * @brief Schema definition for configuration validation
 */
class ConfigSchema {
public:
    ConfigSchema() = default;

    void AddRequired(const std::string& key, const std::string& description = "");
    void AddOptional(const std::string& key, const std::string& description = "");
    void AddValidator(const std::string& key, std::function<ValidationResult(const YAML::Node&)> validator);

    ValidationResult Validate(const YAML::Node& config) const;

private:
    std::vector<ConfigDescriptor> descriptors_;
};

/**
 * @brief Thread-safe configuration loader with layered merging
 *
 * This class manages configuration from multiple sources with clear precedence:
 * 1. RobotDefault - Hardcoded defaults
 * 2. BaseYaml - base.yaml
 * 3. RuntimeYaml - robot_lab/config.yaml
 * 4. LaunchArgs - Launch file parameters
 * 5. RuntimeOverride - Programmatic overrides
 *
 * Higher layers override values from lower layers.
 */
class ConfigLoader {
public:
    ConfigLoader();
    explicit ConfigLoader(const std::string& robot_name);

    /**
     * @brief Load a configuration layer from a YAML file
     * @param layer The layer to load
     * @param file_path Path to the YAML file
     * @return ValidationResult indicating success or failure
     */
    ValidationResult LoadLayerFromFile(ConfigLayer layer, const std::string& file_path);

    /**
     * @brief Load a configuration layer from a YAML node
     * @param layer The layer to load
     * @param node The YAML node containing the configuration
     * @return ValidationResult indicating success or failure
     */
    ValidationResult LoadLayerFromNode(ConfigLayer layer, const YAML::Node& node);

    /**
     * @brief Set a configuration value programmatically (highest priority layer)
     * @param key Configuration key (supports dot notation, e.g., "arm_control.kp")
     * @param value Value to set
     */
    void Set(const std::string& key, const YAML::Node& value);

    /**
     * @brief Get a configuration value with automatic type conversion
     * @tparam T The type to convert to
     * @param key Configuration key (supports dot notation)
     * @param default_value Default value if key not found
     * @return The configuration value or default
     */
    template<typename T>
    T Get(const std::string& key, const T& default_value) const;

    /**
     * @brief Get a configuration value, throwing if not found
     * @tparam T The type to convert to
     * @param key Configuration key
     * @return The configuration value
     * @throws std::runtime_error if key not found
     */
    template<typename T>
    T GetRequired(const std::string& key) const;

    /**
     * @brief Check if a key exists in the merged configuration
     * @param key Configuration key
     * @return True if the key exists
     */
    bool Has(const std::string& key) const;

    /**
     * @brief Validate the current configuration against a schema
     * @param schema The schema to validate against
     * @return ValidationResult indicating success or failure
     */
    ValidationResult Validate(const ConfigSchema& schema) const;

    /**
     * @brief Get the merged configuration node (read-only)
     * @return The merged YAML node
     */
    YAML::Node GetMergedNode() const;

    /**
     * @brief Clear all configuration layers
     */
    void Clear();

    /**
     * @brief Get the robot name
     * @return The robot name
     */
    std::string GetRobotName() const { return robot_name_; }

    /**
     * @brief Set the robot name
     * @param name The robot name
     */
    void SetRobotName(const std::string& name) { robot_name_ = name; }

    /**
     * @brief Create a schema for go2_x5 robot configuration
     * @return ConfigSchema with go2_x5 specific validation rules
     */
    static ConfigSchema CreateGo2X5Schema();

    /**
     * @brief Create a schema for go2 robot configuration
     * @return ConfigSchema with go2 specific validation rules
     */
    static ConfigSchema CreateGo2Schema();

    // Allow ConfigOverrideGuard to access private members
    friend class ConfigOverrideGuard;

private:
    YAML::Node GetLayerNode(ConfigLayer layer) const;
    YAML::Node MergeLayers() const;
    std::vector<std::string> SplitKey(const std::string& key) const;
    YAML::Node GetNestedNode(YAML::Node& root, const std::vector<std::string>& keys) const;

    mutable std::mutex mutex_;
    std::string robot_name_;
    std::map<ConfigLayer, YAML::Node> layers_;
    mutable YAML::Node cached_merged_;
    mutable bool merged_dirty_ = true;
};

/**
 * @brief Helper class for scoped config overrides
 *
 * Automatically restores previous values when destroyed.
 */
class ConfigOverrideGuard {
public:
    ConfigOverrideGuard(ConfigLoader& loader, const std::string& key,
                      const YAML::Node& override_value);
    ~ConfigOverrideGuard();

    // Non-copyable
    ConfigOverrideGuard(const ConfigOverrideGuard&) = delete;
    ConfigOverrideGuard& operator=(const ConfigOverrideGuard&) = delete;

private:
    ConfigLoader& loader_;
    std::string key_;
    YAML::Node previous_value_;
    bool has_previous_;
};

} // namespace RLConfig

#endif // RL_SAR_CONFIG_LOADER_HPP
