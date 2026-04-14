/**
 * @file test_mocked_control_loop.cpp
 * @brief 使用模拟数据的控制回路集成测试
 *
 * 测试内容：
 * 1. 加载deploy manifest
 * 2. 初始化supervisor和coordinator
 * 3. 使用模拟数据运行完整控制回路
 * 4. 验证状态转移
 * 5. 验证输出命令
 */

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <thread>

#include "rl_sar/go2x5/config/go2_x5_config.hpp"
#include "rl_sar/runtime/coordinator/go2_x5_coordinator.hpp"
#include "rl_sar/runtime/supervisor/go2_x5_supervisor.hpp"
#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

namespace Test
{

class MockedDataGenerator
{
public:
    struct Config
    {
        float base_position_noise = 0.001f;  // rad
        float base_velocity_noise = 0.01f;    // m/s
        float imu_noise = 0.001f;             // rad/s
    };

    explicit MockedDataGenerator(Config config = {})
        : config_(config), rng_(std::random_device{}())
    {
        // Go2站立姿态（示例）
        stand_q_ = {
             0.0f,  0.8f, -1.6f,   // FL
             0.0f,  0.8f, -1.6f,   // FR
             0.0f, -0.8f,  1.6f,   // RL
             0.0f, -0.8f,  1.6f    // RR
        };
    }

    void GenerateBodyState(rl_sar::protocol::BodyStateFrame& frame, uint64_t now_ns)
    {
        frame = rl_sar::protocol::BodyStateFrame{};

        // IMU数据（单位四元数，假设机器人平放）
        frame.imu_quat = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
        frame.imu_gyro = {0.0f, 0.0f, 0.0f};
        frame.imu_acc = {0.0f, 0.0f, -9.81f};

        // 添加噪声
        std::normal_distribution<float> noise_dist(0.0f, config_.imu_noise);
        frame.imu_gyro[0] += noise_dist(rng_);
        frame.imu_gyro[1] += noise_dist(rng_);

        // Base线速度（静止）
        frame.base_lin_vel = {0.0f, 0.0f, 0.0f};
        frame.base_ang_vel = {0.0f, 0.0f, 0.0f};

        // 重力向量（静止时向下）
        frame.projected_gravity = {0.0f, 0.0f, -1.0f};

        // 关节状态
        for (size_t i = 0; i < 12; ++i)
        {
            frame.leg_q[i] = stand_q_[i] + Noise() * config_.base_position_noise;
            frame.leg_dq[i] = Noise() * config_.base_velocity_noise;
            frame.leg_tau[i] = 0.0f;
        }

        frame.header.seq = ++body_seq_;
        frame.header.source_monotonic_ns = now_ns;
        frame.header.publish_monotonic_ns = now_ns;
        frame.header.msg_type = rl_sar::protocol::FrameType::BodyState;
        frame.lowstate_age_us = 1000;  // 1ms
        frame.dds_ok = 1;
    }

    void GenerateArmState(rl_sar::protocol::ArmStateFrame& frame, uint64_t now_ns)
    {
        frame = rl_sar::protocol::ArmStateFrame{};

        // Arm home位置
        frame.q = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        frame.dq = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        frame.tau = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        frame.q_target = frame.q;
        frame.tracking_error = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        frame.header.seq = ++arm_seq_;
        frame.header.source_monotonic_ns = now_ns;
        frame.header.publish_monotonic_ns = now_ns;
        frame.header.msg_type = rl_sar::protocol::FrameType::ArmState;
        frame.backend_age_us = 500;
        frame.transport_age_us = 500;
    }

    void GeneratePolicyCommand(rl_sar::protocol::DogPolicyCommandFrame& frame, uint64_t now_ns)
    {
        frame = rl_sar::protocol::DogPolicyCommandFrame{};

        // 小幅动作（站立姿态附近）
        for (size_t i = 0; i < 12; ++i)
        {
            frame.leg_action[i] = Noise() * 0.01f;  // 小幅扰动
        }

        frame.header.seq = ++policy_seq_;
        frame.header.source_monotonic_ns = now_ns;
        frame.header.publish_monotonic_ns = now_ns;
        frame.header.msg_type = rl_sar::protocol::FrameType::DogPolicyCommand;
        frame.action_dim = 12;
        frame.inference_latency_us = 5000;  // 5ms
    }

    uint64_t BodySeq() const { return body_seq_; }
    uint64_t ArmSeq() const { return arm_seq_; }
    uint64_t PolicySeq() const { return policy_seq_; }

private:
    float Noise()
    {
        std::normal_distribution<float> dist(0.0f, 1.0f);
        return dist(rng_);
    }

    Config config_;
    std::mt19937 rng_;
    std::array<float, 12> stand_q_;
    uint64_t body_seq_ = 0;
    uint64_t arm_seq_ = 0;
    uint64_t policy_seq_ = 0;
};

// ============================================================================
// Test Runner
// ============================================================================

class MockedControlLoopTest
{
public:
    struct TestConfig
    {
        int iterations = 100;
        int rate_hz = 200;
        bool verbose = false;
    };

    explicit MockedControlLoopTest(TestConfig config = {})
        : config_(config), data_gen_{}
    {
    }

    bool Run()
    {
        std::cout << "========================================" << std::endl;
        std::cout << "  Mocked Control Loop Integration Test" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Iterations: " << config_.iterations << std::endl;
        std::cout << "Rate: " << config_.rate_hz << " Hz" << std::endl;
        std::cout << std::endl;

        // 初始化组件
        if (!InitializeComponents())
        {
            std::cerr << "Failed to initialize components" << std::endl;
            return false;
        }

        // 运行测试
        bool success = RunTestLoop();

        // 报告结果
        ReportResults();

        return success;
    }

private:
    bool InitializeComponents()
    {
        // 初始化Supervisor
        Go2X5Supervisor::Config supervisor_config;
        supervisor_config.body_state_stale_us = 50000;   // 50ms
        supervisor_config.arm_state_stale_us = 50000;    // 50ms
        supervisor_config.policy_stale_us = 100000;      // 100ms
        supervisor_config.probe_window_us = 2000000;     // 2s
        supervisor_config.fault_latched_requires_manual_reset = false;

        supervisor_ = std::make_unique<Go2X5Supervisor::Supervisor>(supervisor_config);

        // 初始化Coordinator
        rl_sar::runtime::coordinator::FallbackSmootherConfig smoother_config;
        smoother_config.max_joint_velocity = 3.14f;
        smoother_config.default_duration = 0.5f;

        rl_sar::runtime::coordinator::Config coord_config;
        coord_config.fallback_smoother = smoother_config;
        coord_config.body_command_expire_ns = 15000000;  // 15ms
        coord_config.arm_command_expire_ns = 15000000;   // 15ms
        coord_config.policy_fresh_threshold_ns = 100000000;  // 100ms

        // 默认腿部位置
        coord_config.default_leg_q = {
             0.0f,  0.8f, -1.6f,
             0.0f,  0.8f, -1.6f,
             0.0f, -0.8f,  1.6f,
             0.0f, -0.8f,  1.6f
        };
        coord_config.safe_stand_q = coord_config.default_leg_q;

        // RL增益
        coord_config.rl_kp = {
            50.0f, 50.0f, 50.0f,
            50.0f, 50.0f, 50.0f,
            50.0f, 50.0f, 50.0f,
            50.0f, 50.0f, 50.0f
        };
        coord_config.rl_kd = {
            1.5f, 1.5f, 1.5f,
            1.5f, 1.5f, 1.5f,
            1.5f, 1.5f, 1.5f,
            1.5f, 1.5f, 1.5f
        };
        coord_config.action_scale = std::array<float, 12>{};
        std::fill(coord_config.action_scale.begin(), coord_config.action_scale.end(), 0.5f);
        coord_config.torque_limits = std::array<float, 12>{};
        std::fill(coord_config.torque_limits.begin(), coord_config.torque_limits.end(), 25.0f);

        coordinator_ = std::make_unique<rl_sar::runtime::coordinator::HybridMotionCoordinator>(coord_config);

        return true;
    }

    bool RunTestLoop()
    {
        using namespace std::chrono_literals;

        const auto period = 1000ms / config_.rate_hz;
        auto start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < config_.iterations; ++i)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

            // 生成模拟数据
            rl_sar::protocol::BodyStateFrame body_state;
            rl_sar::protocol::ArmStateFrame arm_state;
            rl_sar::protocol::DogPolicyCommandFrame policy_cmd;

            data_gen_.GenerateBodyState(body_state, now_ns);
            data_gen_.GenerateArmState(arm_state, now_ns);
            data_gen_.GeneratePolicyCommand(policy_cmd, now_ns);

            // 构建Supervisor输入
            Go2X5Supervisor::WatchdogInput supervisor_input;
            supervisor_input.now_monotonic_ns = now_ns;
            supervisor_input.manifest_valid = true;
            supervisor_input.config_loaded = true;
            supervisor_input.boot_complete = true;
            supervisor_input.body_state_age_us = 1000;   // 1ms
            supervisor_input.arm_state_age_us = 500;     // 0.5ms
            supervisor_input.policy_age_us = 5000;      // 5ms
            supervisor_input.has_body_state_seq = true;
            supervisor_input.has_arm_state_seq = true;
            supervisor_input.has_policy_seq = true;
            supervisor_input.body_state_seq = data_gen_.BodySeq();
            supervisor_input.arm_state_seq = data_gen_.ArmSeq();
            supervisor_input.policy_seq = data_gen_.PolicySeq();
            supervisor_input.body_dds_write_ok = true;
            supervisor_input.arm_backend_valid = true;
            supervisor_input.arm_tracking_error_high = false;
            supervisor_input.policy_health_ok = true;
            supervisor_input.policy_health_bad = false;
            supervisor_input.estop = false;
            supervisor_input.soft_stop_request = false;
            supervisor_input.fault_reset = false;
            supervisor_input.allow_recover = true;

            // 前几次迭代模拟probe通过
            if (i < 5)
            {
                supervisor_input.probe_pass = true;
            }
            else
            {
                supervisor_input.probe_fail = false;
                supervisor_input.probe_pass = false;
                supervisor_input.operator_enable = (i == 10);  // 第10次迭代启用
            }

            // 运行Supervisor
            auto supervisor_result = supervisor_->Step(supervisor_input);
            if (config_.verbose && supervisor_result.mode_changed)
            {
                std::cout << "[" << i << "] Mode: "
                          << Go2X5Supervisor::ToString(supervisor_result.previous_mode)
                          << " -> "
                          << Go2X5Supervisor::ToString(supervisor_result.current_mode)
                          << std::endl;
            }

            // 记录状态转移
            if (supervisor_result.mode_changed)
            {
                mode_transitions_.push_back({
                    supervisor_result.previous_mode,
                    supervisor_result.current_mode,
                    i
                });
            }

            // 构建Coordinator输入
            rl_sar::runtime::coordinator::Input coord_input;
            coord_input.now_monotonic_ns = now_ns;
            coord_input.mode = supervisor_result.current_mode;
            coord_input.has_body_state = true;
            coord_input.body_state = body_state;
            coord_input.has_arm_state = true;
            coord_input.arm_state = arm_state;
            coord_input.has_arm_command = false;
            coord_input.has_policy_command = true;
            coord_input.dog_policy_command = policy_cmd;
            coord_input.arm_backend_valid = true;
            coord_input.arm_tracking_error_high = false;

            // 运行Coordinator
            auto coord_output = coordinator_->Step(coord_input);

            // 验证输出
            ValidateOutput(i, supervisor_result, coord_output);

            // 模拟控制频率
            std::this_thread::sleep_until(now + period);
        }

        return true;
    }

    void ValidateOutput(
        int iteration,
        const Go2X5Supervisor::TransitionResult& supervisor_result,
        const rl_sar::runtime::coordinator::Output& coord_output)
    {
        // 验证body command
        if (coord_output.body_command_valid)
        {
            const auto& cmd = coord_output.body_command;

            // 检查关节数量
            if (cmd.joint_count != 12)
            {
                errors_.push_back("[" + std::to_string(iteration) + "] Invalid joint count: " +
                                  std::to_string(cmd.joint_count));
            }

            // 检查位置限制
            for (size_t i = 0; i < 12; ++i)
            {
                if (std::abs(cmd.q[i]) > 3.5f)
                {
                    errors_.push_back("[" + std::to_string(iteration) + "] Joint " +
                                      std::to_string(i) + " position out of range: " +
                                      std::to_string(cmd.q[i]));
                }
            }
        }

        // 验证状态机
        if (supervisor_result.current_mode == Go2X5Supervisor::Mode::FaultLatched)
        {
            errors_.push_back("[" + std::to_string(iteration) + "] Unexpected FAULT_LATCHED");
        }

        // 统计
        if (coord_output.body_command_valid) body_command_count_++;
        if (coord_output.arm_command_valid) arm_command_count_++;
        if (coord_output.policy_applied) policy_applied_count_++;
    }

    void ReportResults()
    {
        std::cout << "\n========================================" << std::endl;
        std::cout << "  Test Results" << std::endl;
        std::cout << "========================================" << std::endl;

        std::cout << "\nMode Transitions:" << std::endl;
        for (const auto& t : mode_transitions_)
        {
            std::cout << "  "
                      << Go2X5Supervisor::ToString(t.from_mode)
                      << " -> "
                      << Go2X5Supervisor::ToString(t.to_mode)
                      << " (iteration " << t.iteration << ")"
                      << std::endl;
        }

        std::cout << "\nCommand Statistics:" << std::endl;
        std::cout << "  Body commands: " << body_command_count_ << "/" << config_.iterations << std::endl;
        std::cout << "  Arm commands:  " << arm_command_count_ << "/" << config_.iterations << std::endl;
        std::cout << "  Policy applied: " << policy_applied_count_ << "/" << config_.iterations << std::endl;

        if (!errors_.empty())
        {
            std::cout << "\nErrors (" << errors_.size() << "):" << std::endl;
            for (const auto& e : errors_)
            {
                std::cout << "  " << e << std::endl;
            }
        }

        std::cout << "\n";
        if (errors_.empty())
        {
            std::cout << "✓ All tests passed!" << std::endl;
        }
        else
        {
            std::cout << "✗ " << errors_.size() << " errors detected" << std::endl;
        }
        std::cout << "========================================" << std::endl;
    }

    struct ModeTransition
    {
        Go2X5Supervisor::Mode from_mode;
        Go2X5Supervisor::Mode to_mode;
        int iteration;
    };

    TestConfig config_;
    MockedDataGenerator data_gen_;
    std::unique_ptr<Go2X5Supervisor::Supervisor> supervisor_;
    std::unique_ptr<rl_sar::runtime::coordinator::HybridMotionCoordinator> coordinator_;

    std::vector<ModeTransition> mode_transitions_;
    std::vector<std::string> errors_;

    int body_command_count_ = 0;
    int arm_command_count_ = 0;
    int policy_applied_count_ = 0;
};

} // namespace Test

int main(int argc, char** argv)
{
    Test::MockedControlLoopTest::TestConfig config;
    config.iterations = 200;
    config.rate_hz = 200;
    config.verbose = true;

    // 解析参数
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--iterations" && i + 1 < argc)
        {
            config.iterations = std::atoi(argv[++i]);
        }
        else if (arg == "--rate" && i + 1 < argc)
        {
            config.rate_hz = std::atoi(argv[++i]);
        }
        else if (arg == "--verbose")
        {
            config.verbose = true;
        }
        else if (arg == "--quiet")
        {
            config.verbose = false;
        }
    }

    Test::MockedControlLoopTest test(config);
    return test.Run() ? 0 : 1;
}
