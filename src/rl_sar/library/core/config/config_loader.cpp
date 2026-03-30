 */

#include "config_loader.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>
#include <sstream>

namespace RLConfig {

// ============================================================================
// ConfigSchema Implementation
// ============================================================================

void ConfigSchema::AddRequired(const std::string& key, const std::string& description) {
    descriptors_.emplace_back(key, true, description);
}

void ConfigSchema::AddOptional(const std::string& key, const std::string& description) {
    descriptors_.emplace_back(key, false, description);
}

void ConfigSchema::AddValidator(const std::string& key,
                                std::function<ValidationResult(const YAML::Node&)> validator) {
    descriptors_.emplace_back(key, false);
    descriptors_.back().validator = std::move(validator);
}

ValidationResult ConfigSchema::Validate(const YAML::Node& config) const {
    for (const auto& desc : descriptors_) {
        // Check if key exists using dot notation
        YAML::Node current = config;
        std::string full_path;
        bool path_valid = true;

        std::string key = desc.key;
        size_t pos = 0;
        while ((pos = key.find('.')) != std::string::npos) {
            std::string part = key.substr(0, pos);
            if (!full_path.empty()) full_path += ".";
            full_path += part;

            if (!current || !current.IsMap()) {
                path_valid = false;
                break;
            }
            current = current[part];
            key.erase(0, pos + 1);
        }

        if (!full_path.empty()) full_path += ".";
        full_path += key;

        if (!path_valid || !current) {
            if (desc.is_required) {
                return ValidationResult::Error(
                    "Required configuration key is missing: " + desc.key,
                    desc.key);
            }
            continue;  // Optional key not present, skip
        }

        // Run custom validator if present
        if (desc.validator) {
            ValidationResult result = desc.validator(current);
            if (!result.is_valid) {
                result.error_path = desc.key + (result.error_path.empty() ? "" : "." + result.error_path);
                return result;
            }
        }
    }

    return ValidationResult::Ok();
}

// ============================================================================
// ConfigLoader Implementation
// ============================================================================

ConfigLoader::ConfigLoader()
    : robot_name_("unknown") {}

ConfigLoader::ConfigLoader(const std::string& robot_name)
    : robot_name_(robot_name) {}

ValidationResult ConfigLoader::LoadLayerFromFile(ConfigLayer layer,
                                                 const std::string& file_path) {
    try {
        YAML::Node node = YAML::LoadFile(file_path);
        return LoadLayerFromNode(layer, node);
    } catch (const YAML::Exception& e) {
        return ValidationResult::Error(
            "Failed to load YAML file: " + file_path + " - " + e.what(),
            file_path);
    } catch (const std::exception& e) {
        return ValidationResult::Error(
            "Failed to load file: " + file_path + " - " + e.what(),
            file_path);
    }
}

ValidationResult ConfigLoader::LoadLayerFromNode(ConfigLayer layer,
                                                const YAML::Node& node) {
    if (!node || !node.IsDefined()) {
        return ValidationResult::Error("Invalid YAML node", "");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    layers_[layer] = node;
    merged_dirty_ = true;
    return ValidationResult::Ok();
}

void ConfigLoader::Set(const std::string& key, const YAML::Node& value) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Get the RuntimeOverride layer (creating it if needed)
    YAML::Node& override_layer = layers_[ConfigLayer::RuntimeOverride];

    // Ensure the override layer is a map
    if (!override_layer.IsDefined() || !override_layer.IsMap()) {
        override_layer = YAML::Node(YAML::NodeType::Map);
    }

    // Split the key into parts
    auto keys = SplitKey(key);

    // Navigate and set the value
    // YAML-CPP nodes are reference-counted, so assigning creates a reference
    if (keys.empty()) {
        return;  // Invalid key
    }

    // Build the path and set the value
    YAML::Node current = override_layer;
    for (size_t i = 0; i < keys.size() - 1; ++i) {
        const std::string& k = keys[i];
        // Ensure the intermediate node exists and is a map
        if (!current[k] || !current[k].IsMap()) {
            current[k] = YAML::Node(YAML::NodeType::Map);
        }
        // Move to the next level (this creates a new shared reference)
        current = current[k];
    }
    // Set the final value
    current[keys.back()] = value;

    merged_dirty_ = true;
}

bool ConfigLoader::Has(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);

    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);

    YAML::Node current = merged;
    for (const auto& k : keys) {
        if (!current || !current.IsMap()) {
            return false;
        }
        current = current[k];
        if (!current || !current.IsDefined()) {
            return false;
        }
    }

    return true;
}

YAML::Node ConfigLoader::GetMergedNode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return MergeLayers();
}

ValidationResult ConfigLoader::Validate(const ConfigSchema& schema) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return schema.Validate(MergeLayers());
}

void ConfigLoader::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    layers_.clear();
    merged_dirty_ = true;
}

YAML::Node ConfigLoader::GetLayerNode(ConfigLayer layer) const {
    auto it = layers_.find(layer);
    if (it != layers_.end()) {
        return it->second;
    }
    return YAML::Node();
}

YAML::Node ConfigLoader::MergeLayers() const {
    if (!merged_dirty_ && cached_merged_) {
        return cached_merged_;
    }

    YAML::Node merged;
    for (const auto& [layer, node] : layers_) {
        if (!node || !node.IsDefined()) continue;

        // Merge node into merged (recursively for maps)
        if (node.IsMap()) {
            if (!merged) {
                merged = YAML::Node(YAML::NodeType::Map);
            }
            for (const auto& kv : node) {
                merged[kv.first.Scalar()] = kv.second;
            }
        } else {
            merged = node;
        }
    }

    cached_merged_ = merged;
    merged_dirty_ = false;
    return merged;
}

std::vector<std::string> ConfigLoader::SplitKey(const std::string& key) const {
    std::vector<std::string> result;
    std::stringstream ss(key);
    std::string item;

    while (std::getline(ss, item, '.')) {
        if (!item.empty()) {
            result.push_back(item);
        }
    }

    return result;
}

YAML::Node ConfigLoader::GetNestedNode(YAML::Node& root,
                                       const std::vector<std::string>& keys) const {
    YAML::Node current = root;
    for (const auto& k : keys) {
        if (!current || !current.IsMap()) {
            return YAML::Node();
        }
        current = current[k];
    }
    return current;
}

// ============================================================================
// Template Specializations
// ============================================================================

template<>
int ConfigLoader::Get<int>(const std::string& key, const int& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && !node.IsNull()) {
        try {
            return node.as<int>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
float ConfigLoader::Get<float>(const std::string& key, const float& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && !node.IsNull()) {
        try {
            return node.as<float>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
double ConfigLoader::Get<double>(const std::string& key, const double& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && !node.IsNull()) {
        try {
            return node.as<double>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
bool ConfigLoader::Get<bool>(const std::string& key, const bool& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && !node.IsNull()) {
        try {
            return node.as<bool>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
std::string ConfigLoader::Get<std::string>(const std::string& key,
                                          const std::string& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && node.IsScalar()) {
        try {
            return node.as<std::string>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
std::vector<int> ConfigLoader::Get<std::vector<int>>(const std::string& key,
                                                     const std::vector<int>& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && node.IsSequence()) {
        try {
            return node.as<std::vector<int>>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
std::vector<float> ConfigLoader::Get<std::vector<float>>(const std::string& key,
                                                        const std::vector<float>& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && node.IsSequence()) {
        try {
            return node.as<std::vector<float>>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

template<>
std::vector<std::string> ConfigLoader::Get<std::vector<std::string>>(
    const std::string& key,
    const std::vector<std::string>& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (node && node.IsDefined() && node.IsSequence()) {
        try {
            return node.as<std::vector<std::string>>();
        } catch (...) {
            // Fall through to default
        }
    }
    return default_value;
}

// ============================================================================
// Required versions (throw if not found)
// ============================================================================

template<>
int ConfigLoader::GetRequired<int>(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (!node || !node.IsDefined()) {
        throw std::runtime_error("Required configuration key not found: " + key);
    }

    try {
        return node.as<int>();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to convert key '" + key + "' to int: " + e.what());
    }
}

template<>
float ConfigLoader::GetRequired<float>(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (!node || !node.IsDefined()) {
        throw std::runtime_error("Required configuration key not found: " + key);
    }

    try {
        return node.as<float>();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to convert key '" + key + "' to float: " + e.what());
    }
}

template<>
std::string ConfigLoader::GetRequired<std::string>(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = GetNestedNode(merged, keys);

    if (!node || !node.IsDefined()) {
        throw std::runtime_error("Required configuration key not found: " + key);
    }

    try {
        return node.as<std::string>();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to convert key '" + key + "' to string: " + e.what());
    }
}

// ============================================================================
// Schema Creation Helpers
// ============================================================================

ConfigSchema ConfigLoader::CreateGo2X5Schema() {
    ConfigSchema schema;

    // Required top-level keys
    schema.AddRequired("num_of_dofs", "Total number of degrees of freedom");
    schema.AddRequired("default_dof_pos", "Default joint positions");

    // Arm-specific keys
    schema.AddRequired("arm_command_size", "Number of arm joints");
    schema.AddRequired("arm_control_mode", "Arm control mode (unitree/split/bridge)");

    // Add validators for vector sizes
    schema.AddValidator("num_of_dofs", [](const YAML::Node& node) -> ValidationResult {
        if (!node.IsScalar()) {
            return ValidationResult::Error("num_of_dofs must be a scalar");
        }
        int dof = node.as<int>();
        if (dof <= 0 || dof > 30) {
            return ValidationResult::Error("num_of_dofs must be between 1 and 30");
        }
        return ValidationResult::Ok();
    });

    schema.AddValidator("arm_command_size", [](const YAML::Node& node) -> ValidationResult {
        if (!node.IsScalar()) {
            return ValidationResult::Error("arm_command_size must be a scalar");
        }
        int size = node.as<int>();
        if (size < 0 || size > 12) {
            return ValidationResult::Error("arm_command_size must be between 0 and 12");
        }
        return ValidationResult::Ok();
    });

    schema.AddValidator("dt", [](const YAML::Node& node) -> ValidationResult {
        if (!node.IsScalar()) {
            return ValidationResult::Error("dt must be a scalar");
        }
        float dt = node.as<float>();
        if (dt <= 0.0f || dt > 1.0f) {
            return ValidationResult::Error("dt must be between 0 and 1 second");
        }
        return ValidationResult::Ok();
    });

    return schema;
}

ConfigSchema ConfigLoader::CreateGo2Schema() {
    ConfigSchema schema;

    // Required top-level keys for Go2 (without arm)
    schema.AddRequired("num_of_dofs", "Total number of degrees of freedom");
    schema.AddRequired("default_dof_pos", "Default joint positions");

    // Similar validators but without arm-specific ones
    schema.AddValidator("num_of_dofs", [](const YAML::Node& node) -> ValidationResult {
        if (!node.IsScalar()) {
            return ValidationResult::Error("num_of_dofs must be a scalar");
        }
        int dof = node.as<int>();
        if (dof <= 0 || dof > 30) {
            return ValidationResult::Error("num_of_dofs must be between 1 and 30");
        }
        return ValidationResult::Ok();
    });

    schema.AddValidator("dt", [](const YAML::Node& node) -> ValidationResult {
        if (!node.IsScalar()) {
            return ValidationResult::Error("dt must be a scalar");
        }
        float dt = node.as<float>();
        if (dt <= 0.0f || dt > 1.0f) {
            return ValidationResult::Error("dt must be between 0 and 1 second");
        }
        return ValidationResult::Ok();
    });

    return schema;
}

// ============================================================================
// ConfigOverrideGuard Implementation
// ============================================================================

ConfigOverrideGuard::ConfigOverrideGuard(ConfigLoader& loader,
                                       const std::string& key,
                                       const YAML::Node& override_value)
    : loader_(loader), key_(key), has_previous_(false) {
    // Check if there's an existing value to restore
    {
        std::lock_guard<std::mutex> lock(loader_.mutex_);
        YAML::Node merged = loader_.MergeLayers();
        auto keys = loader_.SplitKey(key_);
        YAML::Node existing = loader_.GetNestedNode(merged, keys);

        if (existing && existing.IsDefined()) {
            previous_value_ = existing;
            has_previous_ = true;
        }
    }

    // Apply the override (Set() will acquire its own lock)
    loader_.Set(key_, override_value);
}

ConfigOverrideGuard::~ConfigOverrideGuard() {
    if (has_previous_) {
        loader_.Set(key_, previous_value_);
    } else {
        // Need to remove the override - this is tricky with YAML
        // For now, we'll just set it to a null node
        loader_.Set(key_, YAML::Node());
    }
}

} // namespace RLConfig
