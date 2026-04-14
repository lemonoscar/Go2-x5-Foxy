/**
 * @file test_observation_snapshot.cpp
 * @brief 生成观测snapshot文件，用于训练-部署一致性验证
 *
 * 用法：
 *   1. 在部署环境运行此工具，生成snapshot
 *   2. 将snapshot传递给训练环境
 *   3. 训练环境验证一致性
 */

#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>

#include "rl_sar/observation/observation_builder.hpp"

namespace Test
{

class SnapshotGenerator
{
public:
    struct Config
    {
        std::string description = "Go2-X5 deployment observation reference";
        std::string policy_id = "go2_x5_dog_only_v1";
        bool use_random_seed = true;
        uint32_t fixed_seed = 42;
    };

    explicit SnapshotGenerator(Config config = {})
        : config_(config)
    {
        if (config_.use_random_seed)
        {
            rng_.seed(std::random_device{}());
        }
        else
        {
            rng_.seed(config_.fixed_seed);
        }
    }

    bool Generate(const std::string& output_path)
    {
        rl_sar::observation::ObservationBuilder::SnapshotFile snapshot;
        snapshot.magic = rl_sar::observation::ObservationBuilder::kSnapshotMagic;
        snapshot.version = rl_sar::observation::ObservationBuilder::kSnapshotVersion;

        // 生成代表性的输入
        GenerateInput(snapshot.input);

        // 创建builder并生成参考观测
        rl_sar::observation::ObservationBuilder builder;
        snapshot.reference_obs = builder.Build(snapshot.input);

        // 填充描述
        std::strncpy(snapshot.description.data(), config_.description.c_str(), 127);
        snapshot.description[127] = '\0';

        // 写入文件
        std::ofstream out(output_path, std::ios::binary);
        if (!out)
        {
            std::cerr << "Failed to create snapshot file: " << output_path << std::endl;
            return false;
        }

        out.write(reinterpret_cast<const char*>(&snapshot), sizeof(snapshot));
        if (!out)
        {
            std::cerr << "Failed to write snapshot file" << std::endl;
            return false;
        }

        // 打印统计信息
        PrintStatistics(snapshot);

        return true;
    }

private:
    void GenerateInput(rl_sar::observation::ObservationBuilder::BuildInput& input)
    {
        std::normal_distribution<float> norm_dist(0.0f, 1.0f);
        std::uniform_real_distribution<float> uniform_dist(-1.0f, 1.0f);

        // Base状态 - 假设机器人平放站立
        input.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
        input.base_ang_vel = {
            norm_dist(rng_) * 0.01f,
            norm_dist(rng_) * 0.01f,
            norm_dist(rng_) * 0.01f
        };
        input.base_lin_vel = {
            norm_dist(rng_) * 0.01f,
            norm_dist(rng_) * 0.01f,
            0.0f  // z方向无速度
        };
        input.gravity_vec = {0.0f, 0.0f, -1.0f};

        // 速度命令
        input.velocity_commands = {
            norm_dist(rng_) * 0.5f,  // x
            norm_dist(rng_) * 0.3f,  // y
            norm_dist(rng_) * 0.5f   // yaw
        };

        // 关节位置 - 站立姿态
        const std::array<float, 18> stand_pose = {
            // FL, FR, RL, RR (hip, thigh, calf)
             0.0f,  0.8f, -1.6f,
             0.0f,  0.8f, -1.6f,
             0.0f, -0.8f,  1.6f,
             0.0f, -0.8f,  1.6f,
            // Arm (shoulder_pan, shoulder_lift, elbow, wrist, wrist_rotation, gripper)
             0.0f, -0.5f, 1.5f, 0.0f, 0.0f, 0.0f
        };

        for (size_t i = 0; i < 18; ++i)
        {
            input.full_joint_pos[i] = stand_pose[i] + norm_dist(rng_) * 0.01f;
            input.full_joint_vel[i] = norm_dist(rng_) * 0.1f;
            input.last_actions[i] = norm_dist(rng_) * 0.5f;
        }

        // Height scan（模拟平面地面）
        for (size_t i = 0; i < 187; ++i)
        {
            input.height_scan[i] = -0.15f + norm_dist(rng_) * 0.02f;  // 地面高度
        }

        // Arm命令
        for (size_t i = 0; i < 6; ++i)
        {
            input.arm_joint_command[i] = norm_dist(rng_) * 0.3f;
        }

        // Gripper命令
        input.gripper_command = uniform_dist(rng_);
    }

    void PrintStatistics(const rl_sar::observation::ObservationBuilder::SnapshotFile& snapshot)
    {
        std::cout << "========================================" << std::endl;
        std::cout << "  Observation Snapshot Generated" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Description: " << snapshot.description.data << std::endl;
        std::cout << "Magic: 0x" << std::hex << snapshot.magic << std::dec << std::endl;
        std::cout << "Version: " << snapshot.version << std::endl;
        std::cout << "\nInput Summary:" << std::endl;
        std::cout << "  Base lin vel: ["
                  << snapshot.input.base_lin_vel[0] << ", "
                  << snapshot.input.base_lin_vel[1] << ", "
                  << snapshot.input.base_lin_vel[2] << "]" << std::endl;
        std::cout << "  Base ang vel: ["
                  << snapshot.input.base_ang_vel[0] << ", "
                  << snapshot.input.base_ang_vel[1] << ", "
                  << snapshot.input.base_ang_vel[2] << "]" << std::endl;
        std::cout << "  Joint pos range: ["
                  << MinValue(snapshot.input.full_joint_pos) << ", "
                  << MaxValue(snapshot.input.full_joint_pos) << "]" << std::endl;
        std::cout << "\nOutput Observation:" << std::endl;
        std::cout << "  Dimension: " << rl_sar::observation::ObservationBuilder::kObservationDim << std::endl;
        std::cout << "  Range: ["
                  << MinValue(snapshot.reference_obs) << ", "
                  << MaxValue(snapshot.reference_obs) << "]" << std::endl;
        std::cout << "  Mean: " << MeanValue(snapshot.reference_obs) << std::endl;
        std::cout << "========================================" << std::endl;
    }

    static float MinValue(const std::array<float, 260>& arr)
    {
        return *std::min_element(arr.begin(), arr.end());
    }

    static float MaxValue(const std::array<float, 260>& arr)
    {
        return *std::max_element(arr.begin(), arr.end());
    }

    static float MeanValue(const std::array<float, 260>& arr)
    {
        float sum = 0.0f;
        for (float v : arr) sum += v;
        return sum / arr.size();
    }

    Config config_;
    std::mt19937 rng_;
};

} // namespace Test

void PrintUsage(const char* program)
{
    std::cout << "Usage: " << program << " [options]\n"
              << "\nOptions:\n"
              << "  --output <path>    Output snapshot path (default: go2_x5_obs.bin)\n"
              << "  --description <s>  Snapshot description\n"
              << "  --seed <n>         Fixed random seed (default: random)\n"
              << "  --help             Show this help\n"
              << std::endl;
}

int main(int argc, char** argv)
{
    std::string output_path = "go2_x5_obs.bin";
    Test::SnapshotGenerator::Config config;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--output" && i + 1 < argc)
        {
            output_path = argv[++i];
        }
        else if (arg == "--description" && i + 1 < argc)
        {
            config.description = argv[++i];
        }
        else if (arg == "--seed" && i + 1 < argc)
        {
            config.fixed_seed = std::stoul(argv[++i]);
            config.use_random_seed = false;
        }
        else if (arg == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else
        {
            std::cerr << "Unknown option: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    std::cout << "Generating observation snapshot: " << output_path << std::endl;

    Test::SnapshotGenerator generator(config);
    return generator.Generate(output_path) ? 0 : 1;
}
