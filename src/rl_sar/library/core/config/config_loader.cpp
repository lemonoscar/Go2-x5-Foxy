#include "library/core/config/config_loader.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>
#include <sstream>

namespace RLConfig {

namespace {

YAML::Node FindNestedNode(const YAML::Node& root,
                          const std::vector<std::string>& keys,
                          const size_t index = 0) {
    if (index >= keys.size()) {
        return root;
    }
    if (!root.IsDefined() || root.IsNull() || !root.IsMap()) {
        return YAML::Node();
    }
    const YAML::Node child = root[keys[index]];
    if (!child.IsDefined() || child.IsNull()) {
        return YAML::Node();
    }
    return FindNestedNode(child, keys, index + 1);
}

std::vector<std::string> SplitDotKey(const std::string& key) {
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

void DeepMergeInto(YAML::Node& target, const YAML::Node& source) {
    if (!source || !source.IsDefined()) {
        return;
    }

    if (!source.IsMap()) {
        target = YAML::Clone(source);
        return;
    }

    if (!target || !target.IsDefined() || !target.IsMap()) {
        target = YAML::Node(YAML::NodeType::Map);
    }

    for (const auto& kv : source) {
        const std::string key = kv.first.as<std::string>();
        const YAML::Node value = kv.second;
        if (value && value.IsMap() && target[key] && target[key].IsMap()) {
            YAML::Node child = target[key];
            DeepMergeInto(child, value);
            target[key] = child;
        } else {
            target[key] = YAML::Clone(value);
        }
    }
}

void SetNestedKey(YAML::Node& root,
                  const std::vector<std::string>& keys,
                  const YAML::Node& value,
                  const size_t index = 0) {
    if (index >= keys.size()) {
        return;
    }
    if (!root || !root.IsDefined() || !root.IsMap()) {
        root = YAML::Node(YAML::NodeType::Map);
    }
    if (index + 1 == keys.size()) {
        root[keys[index]] = YAML::Clone(value);
        return;
    }
    YAML::Node child = root[keys[index]];
    if (!child || !child.IsDefined() || !child.IsMap()) {
        child = YAML::Node(YAML::NodeType::Map);
    }
    SetNestedKey(child, keys, value, index + 1);
    root[keys[index]] = child;
}

bool RemoveNestedKey(YAML::Node& root, const std::vector<std::string>& keys, const size_t index = 0) {
    if (!root || !root.IsDefined() || !root.IsMap() || index >= keys.size()) {
        return false;
    }
    if (index + 1 == keys.size()) {
        return root.remove(keys[index]);
    }

    YAML::Node child = root[keys[index]];
    if (!child || !child.IsDefined() || !child.IsMap()) {
        return false;
    }
    const bool removed = RemoveNestedKey(child, keys, index + 1);
    if (removed && child.IsMap() && child.size() == 0) {
        root.remove(keys[index]);
    }
    return removed;
}

} // namespace

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
        YAML::Node current = FindNestedNode(config, SplitDotKey(desc.key));
        if (!current.IsDefined() || current.IsNull()) {
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
    if (!node.IsDefined() || node.IsNull()) {
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

    if (keys.empty()) {
        return;  // Invalid key
    }

    SetNestedKey(override_layer, keys, value);
    merged_dirty_ = true;
}

bool ConfigLoader::Has(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);

    YAML::Node merged = MergeLayers();
    YAML::Node node = FindNestedNode(merged, SplitKey(key));
    return node.IsDefined() && !node.IsNull();
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

    YAML::Node merged(YAML::NodeType::Map);
    for (const auto& [layer, node] : layers_) {
        if (!node || !node.IsDefined()) continue;
        DeepMergeInto(merged, node);
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
    return FindNestedNode(root, keys);
}

// ============================================================================
// Template Specializations
// ============================================================================

template<>
int ConfigLoader::Get<int>(const std::string& key, const int& default_value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    YAML::Node merged = MergeLayers();
    auto keys = SplitKey(key);
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && !node.IsNull()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && !node.IsNull()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && !node.IsNull()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && !node.IsNull()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && node.IsScalar()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && node.IsSequence()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && node.IsSequence()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (node.IsDefined() && node.IsSequence()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (!node.IsDefined() || node.IsNull()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (!node.IsDefined() || node.IsNull()) {
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
    YAML::Node node = FindNestedNode(merged, keys);

    if (!node.IsDefined() || node.IsNull()) {
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
        YAML::Node existing = FindNestedNode(merged, keys);

        if (existing.IsDefined() && !existing.IsNull()) {
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
        std::lock_guard<std::mutex> lock(loader_.mutex_);
        YAML::Node& override_layer = loader_.layers_[ConfigLayer::RuntimeOverride];
        RemoveNestedKey(override_layer, loader_.SplitKey(key_));
        loader_.merged_dirty_ = true;
    }
}

} // namespace RLConfig
