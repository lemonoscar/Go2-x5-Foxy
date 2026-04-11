#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <cmath>
#include <unistd.h>

#include "../library/core/config/config_loader.hpp"
#include <yaml-cpp/yaml.h>

using namespace RLConfig;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << "\n"; \
            return 1; \
        } \
    } while(0)

#define TEST_FLOAT_EQ(a, b, msg) \
    TEST_CHECK(std::fabs((a) - (b)) < 1e-6f, msg)

std::string CreateTempYamlFile(const std::string& content) {
    char filename_template[] = "/tmp/temp_config_XXXXXX";
    int fd = mkstemp(filename_template);
    if (fd == -1) {
        throw std::runtime_error("Failed to create temporary file");
    }
    close(fd);

    // Add .yaml extension to the filename
    std::string filename = std::string(filename_template) + ".yaml";
    std::rename(filename_template, filename.c_str());

    std::ofstream file(filename);
    file << content;
    file.close();

    return filename;
}

int main() {
    int test_count = 0;
    int passed = 0;

    // Test 1: DefaultValues
    {
        test_count++;
        ConfigLoader loader("test_robot");
        if (loader.Get("unknown_key", 42) == 42 &&
            loader.Get("unknown_key", 3.14f) == 3.14f &&
            loader.Get("unknown_key", true) == true &&
            loader.Get("unknown_key", std::string("default")) == "default") {
            passed++;
            std::cout << "✓ DefaultValues\n";
        } else {
            std::cerr << "✗ DefaultValues\n";
        }
    }

    // Test 2: SetAndGet
    {
        test_count++;
        ConfigLoader loader("test_robot");
        loader.Set("test_int", YAML::Node(123));
        loader.Set("test_float", YAML::Node(45.6f));
        loader.Set("test_bool", YAML::Node(true));
        loader.Set("test_string", YAML::Node("hello"));

        if (loader.Get<int>("test_int", 0) == 123 &&
            loader.Get<float>("test_float", 0.0f) == 45.6f &&
            loader.Get<bool>("test_bool", false) == true &&
            loader.Get<std::string>("test_string", "") == "hello") {
            passed++;
            std::cout << "✓ SetAndGet\n";
        } else {
            std::cerr << "✗ SetAndGet\n";
        }
    }

    // Test 3: NestedKeys
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["arm"]["control"]["kp"] = std::vector<float>{1.0f, 2.0f, 3.0f};
        config["arm"]["control"]["kd"] = std::vector<float>{0.1f, 0.2f, 0.3f};
        loader.LoadLayerFromNode(ConfigLayer::RuntimeYaml, config);

        auto kp = loader.Get<std::vector<float>>("arm.control.kp", {});
        auto kd = loader.Get<std::vector<float>>("arm.control.kd", {});

        if (kp.size() == 3 && kp[0] == 1.0f && kp[1] == 2.0f && kp[2] == 3.0f &&
            kd.size() == 3 && kd[0] == 0.1f && kd[1] == 0.2f && kd[2] == 0.3f) {
            passed++;
            std::cout << "✓ NestedKeys\n";
        } else {
            std::cerr << "✗ NestedKeys\n";
        }
    }

    // Test 4: LayerPrecedence
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node base;
        base["value"] = 1;
        base["only_base"] = "base";
        loader.LoadLayerFromNode(ConfigLayer::BaseYaml, base);

        YAML::Node runtime;
        runtime["value"] = 2;
        runtime["only_runtime"] = "runtime";
        loader.LoadLayerFromNode(ConfigLayer::RuntimeYaml, runtime);

        loader.Set("value", YAML::Node(3));

        if (loader.Get<int>("value", 0) == 3 &&
            loader.Get<std::string>("only_base", "") == "base" &&
            loader.Get<std::string>("only_runtime", "") == "runtime") {
            passed++;
            std::cout << "✓ LayerPrecedence\n";
        } else {
            std::cerr << "✗ LayerPrecedence\n";
        }
    }

    // Test 5: LaunchArgsOverrideYaml
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["dt"] = 0.002f;
        config["control_mode"] = "position";
        loader.LoadLayerFromNode(ConfigLayer::BaseYaml, config);

        loader.Set("dt", YAML::Node(0.001f));

        if (loader.Get<float>("dt", 0.0f) == 0.001f &&
            loader.Get<std::string>("control_mode", "") == "position") {
            passed++;
            std::cout << "✓ LaunchArgsOverrideYaml\n";
        } else {
            std::cerr << "✗ LaunchArgsOverrideYaml\n";
        }
    }

    // Test 6: LoadFromFile
    {
        test_count++;
        ConfigLoader loader("test_robot");
        std::string yaml_content = R"(
num_of_dofs: 18
dt: 0.002
default_dof_pos: [0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
observations:
  - ang_vel
  - gravity_vec
  - commands
  - dof_pos
  - dof_vel
arm_control_mode: split
)";

        std::string filename = CreateTempYamlFile(yaml_content);
        auto result = loader.LoadLayerFromFile(ConfigLayer::BaseYaml, filename);

        if (result.is_valid &&
            loader.Get<int>("num_of_dofs", 0) == 18 &&
            loader.Get<float>("dt", 0.0f) == 0.002f &&
            loader.Get<std::string>("arm_control_mode", "") == "split") {
            passed++;
            std::cout << "✓ LoadFromFile\n";
        } else {
            std::cerr << "✗ LoadFromFile: " << result.error_message << "\n";
        }

        std::remove(filename.c_str());
    }

    // Test 7: LoadFromInvalidFile
    {
        test_count++;
        ConfigLoader loader("test_robot");
        auto result = loader.LoadLayerFromFile(ConfigLayer::BaseYaml, "nonexistent_file.yaml");

        if (!result.is_valid) {
            passed++;
            std::cout << "✓ LoadFromInvalidFile\n";
        } else {
            std::cerr << "✗ LoadFromInvalidFile\n";
        }
    }

    // Test 8: ValidateWithValidConfig
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["num_of_dofs"] = 18;
        config["arm_command_size"] = 6;
        config["dt"] = 0.002f;
        config["arm_control_mode"] = "split";
        config["default_dof_pos"] = std::vector<float>(18, 0.0f);

        loader.LoadLayerFromNode(ConfigLayer::BaseYaml, config);

        auto schema = ConfigLoader::CreateGo2X5Schema();
        auto result = loader.Validate(schema);

        if (result.is_valid) {
            passed++;
            std::cout << "✓ ValidateWithValidConfig\n";
        } else {
            std::cerr << "✗ ValidateWithValidConfig: " << result.error_message << "\n";
        }
    }

    // Test 9: ValidateWithMissingRequiredKey
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["num_of_dofs"] = 18;
        config["dt"] = 0.002f;
        config["default_dof_pos"] = std::vector<float>(18, 0.0f);

        loader.LoadLayerFromNode(ConfigLayer::BaseYaml, config);

        auto schema = ConfigLoader::CreateGo2X5Schema();
        auto result = loader.Validate(schema);

        if (!result.is_valid && result.error_message.find("arm_command_size") != std::string::npos) {
            passed++;
            std::cout << "✓ ValidateWithMissingRequiredKey\n";
        } else {
            std::cerr << "✗ ValidateWithMissingRequiredKey\n";
        }
    }

    // Test 10: HasKey
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["existing_key"] = 42;
        config["nested"]["value"] = 100;

        loader.LoadLayerFromNode(ConfigLayer::BaseYaml, config);

        if (loader.Has("existing_key") &&
            loader.Has("nested.value") &&
            !loader.Has("nonexistent_key") &&
            !loader.Has("nested.nonexistent")) {
            passed++;
            std::cout << "✓ HasKey\n";
        } else {
            std::cerr << "✗ HasKey\n";
        }
    }

    // Test 11: GetVectorFloat
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["floats"] = std::vector<float>{1.1f, 2.2f, 3.3f};

        loader.LoadLayerFromNode(ConfigLayer::RuntimeYaml, config);

        auto result = loader.Get<std::vector<float>>("floats", {});

        if (result.size() == 3 && result[0] == 1.1f && result[1] == 2.2f && result[2] == 3.3f) {
            passed++;
            std::cout << "✓ GetVectorFloat\n";
        } else {
            std::cerr << "✗ GetVectorFloat\n";
        }
    }

    // Test 12: GetVectorString
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["strings"] = std::vector<std::string>{"one", "two", "three"};

        loader.LoadLayerFromNode(ConfigLayer::RuntimeYaml, config);

        auto result = loader.Get<std::vector<std::string>>("strings", {});

        if (result.size() == 3 && result[0] == "one" && result[1] == "two" && result[2] == "three") {
            passed++;
            std::cout << "✓ GetVectorString\n";
        } else {
            std::cerr << "✗ GetVectorString\n";
        }
    }

    // Test 13: ClearRemovesAllLayers
    {
        test_count++;
        ConfigLoader loader("test_robot");
        loader.Set("key1", YAML::Node(1));
        loader.Set("key2", YAML::Node(2));

        bool had_keys = loader.Has("key1") && loader.Has("key2");
        loader.Clear();
        bool no_keys = !loader.Has("key1") && !loader.Has("key2");

        if (had_keys && no_keys) {
            passed++;
            std::cout << "✓ ClearRemovesAllLayers\n";
        } else {
            std::cerr << "✗ ClearRemovesAllLayers\n";
        }
    }

    // Test 14: ConfigOverrideGuardRestoresValue
    {
        test_count++;
        ConfigLoader loader("test_robot");
        loader.Set("key", YAML::Node(10));

        {
            ConfigOverrideGuard guard(loader, "key", YAML::Node(20));
            if (loader.Get<int>("key", 0) != 20) {
                std::cerr << "✗ ConfigOverrideGuardRestoresValue (override)\n";
            } else {
                // After guard scope
            }
        }

        if (loader.Get<int>("key", 0) == 10) {
            passed++;
            std::cout << "✓ ConfigOverrideGuardRestoresValue\n";
        } else {
            std::cerr << "✗ ConfigOverrideGuardRestoresValue\n";
        }
    }

    // Test 15: ConfigOverrideGuardWithNewKey
    {
        test_count++;
        ConfigLoader loader("test_robot");

        bool not_has_before = !loader.Has("new_key");
        {
            ConfigOverrideGuard guard(loader, "new_key", YAML::Node(30));
        }
        bool not_has_after = !loader.Has("new_key");

        if (not_has_before && not_has_after) {
            passed++;
            std::cout << "✓ ConfigOverrideGuardWithNewKey\n";
        } else {
            std::cerr << "✗ ConfigOverrideGuardWithNewKey\n";
        }
    }

    // Test 16: SetAndGetRobotName
    {
        test_count++;
        ConfigLoader loader("test_robot");

        bool is_test_robot = (loader.GetRobotName() == "test_robot");
        loader.SetRobotName("go2_x5");
        bool is_go2_x5 = (loader.GetRobotName() == "go2_x5");

        if (is_test_robot && is_go2_x5) {
            passed++;
            std::cout << "✓ SetAndGetRobotName\n";
        } else {
            std::cerr << "✗ SetAndGetRobotName\n";
        }
    }

    // Test 17: ConcurrentReads
    {
        test_count++;
        ConfigLoader loader("test_robot");
        YAML::Node config;
        config["value"] = 123;
        config["array"] = std::vector<int>{1, 2, 3, 4, 5};
        loader.LoadLayerFromNode(ConfigLayer::BaseYaml, config);

        const int num_threads = 10;
        const int iterations = 100;
        std::vector<std::thread> threads;

        for (int t = 0; t < num_threads; ++t) {
            threads.emplace_back([&loader, iterations]() {
                for (int i = 0; i < iterations; ++i) {
                    loader.Get<int>("value", 0);
                    loader.Get<std::vector<int>>("array", {});
                }
            });
        }

        for (auto& thread : threads) {
            thread.join();
        }

        passed++;
        std::cout << "✓ ConcurrentReads\n";
    }

    // Test 18: ConcurrentWrites
    {
        test_count++;
        ConfigLoader loader("test_robot");

        const int num_threads = 5;
        const int iterations = 100;
        std::vector<std::thread> threads;

        for (int t = 0; t < num_threads; ++t) {
            threads.emplace_back([&loader, t, iterations]() {
                for (int i = 0; i < iterations; ++i) {
                    std::string key = "thread_" + std::to_string(t);
                    loader.Set(key, YAML::Node(i));
                }
            });
        }

        for (auto& thread : threads) {
            thread.join();
        }

        // Verify all writes completed
        bool all_ok = true;
        for (int t = 0; t < num_threads; ++t) {
            std::string key = "thread_" + std::to_string(t);
            if (loader.Get<int>(key, -1) != iterations - 1) {
                all_ok = false;
                break;
            }
        }

        if (all_ok) {
            passed++;
            std::cout << "✓ ConcurrentWrites\n";
        } else {
            std::cerr << "✗ ConcurrentWrites\n";
        }
    }

    // Test 19: LoadScopedGo2X5BaseFile
    {
        test_count++;
        ConfigLoader loader("go2_x5");
        std::string yaml_content = R"(
go2_x5:
  num_of_dofs: 18
  dt: 0.005
  default_dof_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
  arm_control_mode: split
)";

        std::string filename = CreateTempYamlFile(yaml_content);
        auto result = loader.LoadLayerFromScopedFile(ConfigLayer::BaseYaml, filename, "go2_x5");

        if (result.is_valid &&
            loader.Get<int>("num_of_dofs", 0) == 18 &&
            loader.Get<std::vector<int>>("joint_mapping", {}).size() == 18 &&
            loader.Get<std::string>("arm_control_mode", "") == "split") {
            passed++;
            std::cout << "✓ LoadScopedGo2X5BaseFile\n";
        } else {
            std::cerr << "✗ LoadScopedGo2X5BaseFile: " << result.error_message << "\n";
        }

        std::remove(filename.c_str());
    }

    // Test 20: LoadScopedGo2X5RuntimeFilesAndValidate
    {
        test_count++;
        ConfigLoader loader("go2_x5");
        std::string base_yaml = R"(
go2_x5:
  num_of_dofs: 18
  default_dof_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
  arm_control_mode: split
)";
        std::string runtime_yaml = R"(
go2_x5/robot_lab:
  model_name: "policy.pt"
  num_observations: 260
  observations: ["lin_vel", "ang_vel"]
  fixed_cmd_x: 0.5
)";

        std::string base_filename = CreateTempYamlFile(base_yaml);
        std::string runtime_filename = CreateTempYamlFile(runtime_yaml);

        auto base_result = loader.LoadLayerFromScopedFile(ConfigLayer::BaseYaml, base_filename, "go2_x5");
        auto runtime_result = loader.LoadLayerFromScopedFile(
            ConfigLayer::RuntimeYaml, runtime_filename, "go2_x5/robot_lab");
        loader.Set("dt", YAML::Node(0.005f));
        loader.Set("arm_command_size", YAML::Node(6));
        loader.Set("arm_joint_count", YAML::Node(6));
        loader.Set("arm_joint_start_index", YAML::Node(12));

        const auto schema_result = loader.Validate(ConfigLoader::CreateGo2X5Schema());

        if (base_result.is_valid &&
            runtime_result.is_valid &&
            schema_result.is_valid &&
            loader.Get<int>("num_of_dofs", 0) == 18 &&
            loader.Get<std::vector<int>>("joint_mapping", {}).size() == 18 &&
            std::fabs(loader.Get<float>("fixed_cmd_x", 0.0f) - 0.5f) < 1e-6f) {
            passed++;
            std::cout << "✓ LoadScopedGo2X5RuntimeFilesAndValidate\n";
        } else {
            std::cerr << "✗ LoadScopedGo2X5RuntimeFilesAndValidate: "
                      << base_result.error_message << " | "
                      << runtime_result.error_message << " | "
                      << schema_result.error_message << "\n";
        }

        std::remove(base_filename.c_str());
        std::remove(runtime_filename.c_str());
    }

    std::cout << "\n========================\n";
    std::cout << "Test Results: " << passed << "/" << test_count << " passed\n";

    return (passed == test_count) ? 0 : 1;
}
