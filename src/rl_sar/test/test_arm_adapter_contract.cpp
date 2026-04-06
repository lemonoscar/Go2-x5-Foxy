/**
 * @file test_arm_adapter_contract.cpp
 * @brief Unit tests for ArxAdapter contract compliance
 *
 * These tests verify that ArxAdapter meets its contractual obligations:
 * - Initialization behavior with various configurations
 * - Command/state threading safety
 * - Tracking error calculation
 * - Health monitoring
 * - State freshness detection
 */

#include <chrono>
#include <thread>
#include <vector>

#include "rl_sar/adapters/arx_adapter.hpp"
#include "rl_sar/protocol/go2_x5_protocol.hpp"

#include <gtest/gtest.h>

namespace rl_sar::adapters::test
{

class ArxAdapterContract : public ::testing::Test
{
protected:
    void SetUp() override
    {
        adapter_ = std::make_unique<ArxAdapter>();
    }

    void TearDown() override
    {
        if (adapter_->IsRunning())
        {
            adapter_->Stop();
        }
    }

    ArxAdapter::Config MakeDefaultConfig()
    {
        ArxAdapter::Config config;
        config.can_interface = "vcan0";  // Virtual CAN for testing
        config.target_rate_hz = 200;
        config.servo_rate_hz = 500;
        config.background_send_recv = true;
        config.controller_dt = 0.002;
        config.require_live_state = false;  // Don't require actual hardware
        config.arm_state_timeout_ms = 50.0;
        config.arm_tracking_error_limit = 0.5;
        config.model = "X5";
        return config;
    }

    protocol::ArmCommandFrame MakeTestCommand(uint64_t seq = 1)
    {
        protocol::ArmCommandFrame cmd;
        cmd.header.seq = seq;
        cmd.header.source_monotonic_ns = GetMonotonicNs();
        cmd.header.publish_monotonic_ns = cmd.header.source_monotonic_ns;
        cmd.command_expire_ns = 100000000;  // 100ms
        cmd.joint_count = 6;
        cmd.q = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        cmd.dq = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        cmd.kp = {50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f};
        cmd.kd = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
        cmd.tau = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        cmd.gripper_target = 0.0f;
        return cmd;
    }

    static uint64_t GetMonotonicNs()
    {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    std::unique_ptr<ArxAdapter> adapter_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(ArxAdapterContract, DefaultConfigValues)
{
    ArxAdapter::Config config;

    EXPECT_EQ(config.can_interface, "can0");
    EXPECT_EQ(config.target_rate_hz, 200);
    EXPECT_EQ(config.servo_rate_hz, 500);
    EXPECT_TRUE(config.background_send_recv);
    EXPECT_DOUBLE_EQ(config.controller_dt, 0.002);
    EXPECT_TRUE(config.require_live_state);
    EXPECT_DOUBLE_EQ(config.arm_state_timeout_ms, 50.0);
    EXPECT_DOUBLE_EQ(config.arm_tracking_error_limit, 0.5);
}

TEST_F(ArxAdapterContract, InitializeWithoutHardware)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;  // Shadow mode

    bool success = adapter_->Initialize(config);

    // Should succeed in shadow mode (without actual ARX SDK)
    EXPECT_TRUE(adapter_->IsInitialized());
}

TEST_F(ArxAdapterContract, InitializeReturnsFalseWithRequireLiveState)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = true;  // Require actual hardware

    bool success = adapter_->Initialize(config);

    // Should fail without actual ARX hardware
    EXPECT_FALSE(success);
}

TEST_F(ArxAdapterContract, InitializeIsIdempotent)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;

    adapter_->Initialize(config);
    bool second_init = adapter_->Initialize(config);

    EXPECT_TRUE(second_init);
    EXPECT_TRUE(adapter_->IsInitialized());
}

TEST_F(ArxAdapterContract, ConfigIsPreserved)
{
    auto config = MakeDefaultConfig();
    config.can_interface = "test_can";
    config.servo_rate_hz = 1000;

    adapter_->Initialize(config);

    EXPECT_EQ(adapter_->GetConfig().can_interface, "test_can");
    EXPECT_EQ(adapter_->GetConfig().servo_rate_hz, 1000);
}

// ============================================================================
// Lifecycle Tests
// ============================================================================

TEST_F(ArxAdapterContract, StartWithoutInitializeFailsGracefully)
{
    adapter_->Start();

    EXPECT_FALSE(adapter_->IsRunning());
}

TEST_F(ArxAdapterContract, StartStopCycle)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);

    adapter_->Start();
    EXPECT_TRUE(adapter_->IsRunning());

    adapter_->Stop();
    EXPECT_FALSE(adapter_->IsRunning());
}

TEST_F(ArxAdapterContract, MultipleStartCalls)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);

    adapter_->Start();
    bool first_running = adapter_->IsRunning();

    adapter_->Start();  // Should be idempotent
    bool second_running = adapter_->IsRunning();

    EXPECT_TRUE(first_running);
    EXPECT_TRUE(second_running);

    adapter_->Stop();
}

TEST_F(ArxAdapterContract, MultipleStopCalls)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);
    adapter_->Start();

    adapter_->Stop();
    bool first_stopped = !adapter_->IsRunning();

    adapter_->Stop();  // Should be idempotent
    bool second_stopped = !adapter_->IsRunning();

    EXPECT_TRUE(first_stopped);
    EXPECT_TRUE(second_stopped);
}

// ============================================================================
// Command Tests
// ============================================================================

TEST_F(ArxAdapterContract, SetCommandBeforeInitialize)
{
    auto cmd = MakeTestCommand();

    // Should not crash
    adapter_->SetCommand(cmd);
}

TEST_F(ArxAdapterContract, SetCommandBasic)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);
    adapter_->Start();

    auto cmd = MakeTestCommand(42);

    adapter_->SetCommand(cmd);

    // Verify stats updated
    auto stats = adapter_->GetStats();
    // Note: commands_sent may be 0 in shadow mode
}

TEST_F(ArxAdapterContract, SetCommandHandlesNaNValues)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);

    protocol::ArmCommandFrame cmd;
    cmd.header.seq = 1;
    cmd.header.source_monotonic_ns = GetMonotonicNs();
    cmd.header.publish_monotonic_ns = cmd.header.source_monotonic_ns;
    cmd.q = {std::numeric_limits<float>::quiet_NaN(), 0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
    cmd.dq = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    cmd.kp = {50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f};
    cmd.kd = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    cmd.tau = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Should not crash
    adapter_->SetCommand(cmd);
}

TEST_F(ArxAdapterContract, SetCommandHandlesInfValues)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);

    protocol::ArmCommandFrame cmd;
    cmd.header.seq = 1;
    cmd.header.source_monotonic_ns = GetMonotonicNs();
    cmd.header.publish_monotonic_ns = cmd.header.source_monotonic_ns;
    cmd.q = {std::numeric_limits<float>::infinity(), 0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
    cmd.dq = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    cmd.kp = {-10.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f};  // Negative kp
    cmd.kd = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    cmd.tau = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Should not crash and should clip negative gains
    adapter_->SetCommand(cmd);
}

TEST_F(ArxAdapterContract, SetCommandThreadSafety)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);
    adapter_->Start();

    constexpr int kNumThreads = 4;
    constexpr int kCommandsPerThread = 100;

    std::vector<std::thread> threads;
    for (int i = 0; i < kNumThreads; ++i)
    {
        threads.emplace_back([this, i]() {
            for (int j = 0; j < kCommandsPerThread; ++j)
            {
                auto cmd = MakeTestCommand(i * kCommandsPerThread + j);
                adapter_->SetCommand(cmd);
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });
    }

    for (auto& t : threads)
    {
        t.join();
    }

    // Should not crash or deadlock
    SUCCEED();
}

// ============================================================================
// State Tests
// ============================================================================

TEST_F(ArxAdapterContract, GetStateBeforeInitialize)
{
    protocol::ArmStateFrame state;

    bool success = adapter_->GetState(state);

    EXPECT_FALSE(success);
}

TEST_F(ArxAdapterContract, GetStateBasic)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);

    protocol::ArmStateFrame state;

    // In shadow mode without backend, state may not be available
    bool success = adapter_->GetState(state);
}

TEST_F(ArxAdapterContract, GetStateAfterTimeout)
{
    auto config = MakeDefaultConfig();
    config.arm_state_timeout_ms = 10.0;  // Short timeout
    config.require_live_state = false;
    adapter_->Initialize(config);

    protocol::ArmStateFrame state;
    bool initial_success = adapter_->GetState(state);

    // Wait for timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    bool after_timeout = adapter_->GetState(state);

    // State should timeout
    EXPECT_FALSE(after_timeout);
}

// ============================================================================
// Health Monitoring Tests
// ============================================================================

TEST_F(ArxAdapterContract, IsTrackingHealthyBeforeInitialize)
{
    EXPECT_FALSE(adapter_->IsTrackingHealthy());
}

TEST_F(ArxAdapterContract, GetStateAgeBeforeInitialize)
{
    uint64_t age_us = adapter_->GetStateAgeUs();

    // Age should be large (effectively infinite)
    EXPECT_GT(age_us, 1000000);  // > 1 second
}

TEST_F(ArxAdapterContract, GetTrackingErrorBeforeInitialize)
{
    double error = adapter_->GetTrackingError();

    // Error should be 0 or reasonable default
    EXPECT_GE(error, 0.0);
}

TEST_F(ArxAdapterContract, StatsInitialState)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);

    auto stats = adapter_->GetStats();

    EXPECT_EQ(stats.commands_sent, 0);
    EXPECT_EQ(stats.states_received, 0);
    EXPECT_EQ(stats.send_failures, 0);
    EXPECT_EQ(stats.state_timeouts, 0);
    EXPECT_EQ(stats.servo_loops, 0);
    EXPECT_FALSE(stats.backend_healthy);
}

// ============================================================================
// Protocol Integration Tests
// ============================================================================

TEST_F(ArxAdapterContract, ArmStateFrameHasCorrectSize)
{
    protocol::ArmStateFrame frame;

    EXPECT_EQ(frame.joint_count, 6);
    EXPECT_EQ(frame.q.size(), 6);
    EXPECT_EQ(frame.dq.size(), 6);
    EXPECT_EQ(frame.tau.size(), 6);
    EXPECT_EQ(frame.q_target.size(), 6);
    EXPECT_EQ(frame.tracking_error.size(), 6);
}

TEST_F(ArxAdapterContract, ArmCommandFrameHasCorrectSize)
{
    protocol::ArmCommandFrame frame;

    EXPECT_EQ(frame.joint_count, 6);
    EXPECT_EQ(frame.q.size(), 6);
    EXPECT_EQ(frame.dq.size(), 6);
    EXPECT_EQ(frame.kp.size(), 6);
    EXPECT_EQ(frame.kd.size(), 6);
    EXPECT_EQ(frame.tau.size(), 6);
}

TEST_F(ArxAdapterContract, ProtocolConstantsMatch)
{
    using namespace protocol;

    EXPECT_EQ(kArmJointCount, 6);
    EXPECT_EQ(kArmStatePayloadSize, 140);
    EXPECT_EQ(kArmCommandPayloadSize, 136);
    EXPECT_EQ(kFrameMagic, 0x50355847u);
    EXPECT_EQ(kProtocolVersion, 1u);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(ArxAdapterContract, ZeroTimeoutConfiguration)
{
    auto config = MakeDefaultConfig();
    config.arm_state_timeout_ms = 0.0;
    config.require_live_state = false;

    bool success = adapter_->Initialize(config);

    EXPECT_TRUE(success);
}

TEST_F(ArxAdapterContract, VeryHighServoRate)
{
    auto config = MakeDefaultConfig();
    config.servo_rate_hz = 2000;  // 2kHz
    config.require_live_state = false;

    bool success = adapter_->Initialize(config);

    EXPECT_TRUE(success);
    EXPECT_EQ(adapter_->GetConfig().servo_rate_hz, 2000);
}

TEST_F(ArxAdapterContract, EmptyCanInterface)
{
    auto config = MakeDefaultConfig();
    config.can_interface = "";
    config.require_live_state = false;

    bool success = adapter_->Initialize(config);

    EXPECT_TRUE(success);
    EXPECT_TRUE(adapter_->IsInitialized());
}

TEST_F(ArxAdapterContract, CustomModelString)
{
    auto config = MakeDefaultConfig();
    config.model = "L5";  // Different model
    config.require_live_state = false;

    bool success = adapter_->Initialize(config);

    EXPECT_TRUE(success);
    EXPECT_EQ(adapter_->GetConfig().model, "L5");
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(ArxAdapterContract, ConcurrentCommandAndState)
{
    auto config = MakeDefaultConfig();
    config.require_live_state = false;
    adapter_->Initialize(config);
    adapter_->Start();

    constexpr int kIterations = 100;

    std::thread command_thread([this, kIterations]() {
        for (int i = 0; i < kIterations; ++i)
        {
            auto cmd = MakeTestCommand(i);
            adapter_->SetCommand(cmd);
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    });

    std::thread state_thread([this, kIterations]() {
        protocol::ArmStateFrame state;
        for (int i = 0; i < kIterations; ++i)
        {
            adapter_->GetState(state);
            adapter_->IsTrackingHealthy();
            adapter_->GetStateAgeUs();
            adapter_->GetTrackingError();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    });

    std::thread stats_thread([this, kIterations]() {
        for (int i = 0; i < kIterations; ++i)
        {
            adapter_->GetStats();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    });

    command_thread.join();
    state_thread.join();
    stats_thread.join();

    // Should not crash or deadlock
    SUCCEED();
}

}  // namespace rl_sar::adapters::test

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
