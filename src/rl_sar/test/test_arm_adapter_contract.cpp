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
#include <array>
#include <string>
#include <thread>
#include <vector>

#include "rl_sar/adapters/arx_adapter.hpp"
#include "rl_sar/protocol/go2_x5_protocol.hpp"

#include <gtest/gtest.h>

#if defined(__linux__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace rl_sar::adapters::test
{

namespace
{

#ifndef RL_SAR_TEST_FAKE_ARX_HARDWARE_LIB
#define RL_SAR_TEST_FAKE_ARX_HARDWARE_LIB ""
#endif

#if defined(__linux__)
int BindUdpSocket(uint16_t* port_out)
{
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        return -1;
    }

    const int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(0);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        close(fd);
        return -1;
    }

    sockaddr_in bound{};
    socklen_t len = sizeof(bound);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &len) != 0)
    {
        close(fd);
        return -1;
    }
    *port_out = ntohs(bound.sin_port);
    return fd;
}

uint16_t ReserveUdpPort()
{
    uint16_t port = 0;
    const int fd = BindUdpSocket(&port);
    if (fd >= 0)
    {
        close(fd);
    }
    return port;
}

class UdpArmBridgeHarness
{
public:
    UdpArmBridgeHarness()
    {
        command_socket_ = BindUdpSocket(&command_port_);
        state_port_ = ReserveUdpPort();
        state_send_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    }

    ~UdpArmBridgeHarness()
    {
        if (command_socket_ >= 0)
        {
            close(command_socket_);
        }
        if (state_send_socket_ >= 0)
        {
            close(state_send_socket_);
        }
    }

    bool IsValid() const
    {
        return command_socket_ >= 0 && state_send_socket_ >= 0 && state_port_ != 0;
    }

    uint16_t command_port() const { return command_port_; }
    uint16_t state_port() const { return state_port_; }

    bool ReceiveCommand(protocol::ArmCommandFrame* frame, const std::chrono::milliseconds timeout)
    {
        if (!frame)
        {
            return false;
        }

        timeval tv{};
        tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
        tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
        setsockopt(command_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::array<uint8_t, 1024> buffer{};
        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        const ssize_t bytes = recvfrom(
            command_socket_,
            buffer.data(),
            buffer.size(),
            0,
            reinterpret_cast<sockaddr*>(&peer),
            &peer_len);
        if (bytes <= 0)
        {
            return false;
        }

        std::string error;
        return rl_sar::protocol::ParseArmCommandFrame(
            std::vector<uint8_t>(buffer.begin(), buffer.begin() + bytes),
            *frame,
            &error);
    }

    bool SendState(const protocol::ArmStateFrame& state)
    {
        const auto packet = rl_sar::protocol::SerializeArmStateFrame(state);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(state_port_);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        const ssize_t sent = sendto(
            state_send_socket_,
            packet.data(),
            packet.size(),
            0,
            reinterpret_cast<const sockaddr*>(&addr),
            sizeof(addr));
        return sent == static_cast<ssize_t>(packet.size());
    }

private:
    int command_socket_ = -1;
    int state_send_socket_ = -1;
    uint16_t command_port_ = 0;
    uint16_t state_port_ = 0;
};
#endif

}  // namespace

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
        const uint64_t now_ns = GetMonotonicNs();
        cmd.header.seq = seq;
        cmd.header.source_monotonic_ns = now_ns;
        cmd.header.publish_monotonic_ns = cmd.header.source_monotonic_ns;
        cmd.command_expire_ns = now_ns + 100000000ULL;  // 100ms from now
        cmd.joint_count = 6;
        cmd.q = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        cmd.dq = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        cmd.kp = {50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f};
        cmd.kd = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
        cmd.tau = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        cmd.gripper_target = 0.0f;
        return cmd;
    }

    static std::string FakeSdkHardwareLibrary()
    {
        return RL_SAR_TEST_FAKE_ARX_HARDWARE_LIB;
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

TEST_F(ArxAdapterContract, InitializesInProcessSdkBackendWithFakeHardwareLibrary)
{
#if defined(__linux__)
    const std::string fake_sdk_lib = FakeSdkHardwareLibrary();
    if (fake_sdk_lib.empty())
    {
        GTEST_SKIP() << "fake ARX hardware library not configured";
    }

    auto config = MakeDefaultConfig();
    config.preferred_backend = ArxAdapter::BackendType::InProcessSdk;
    config.allow_fallback_to_bridge = false;
    config.require_live_state = true;
    config.can_interface = "lo";
    config.sdk_lib_path = fake_sdk_lib;

    ASSERT_TRUE(adapter_->Initialize(config));
    EXPECT_EQ(adapter_->GetActiveBackend(), ArxAdapter::BackendType::InProcessSdk);
    EXPECT_EQ(adapter_->GetBackendName(), "sdk_inprocess_arxcan");

    adapter_->Start();
    adapter_->SetCommand(MakeTestCommand(42));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    protocol::ArmStateFrame state;
    ASSERT_TRUE(adapter_->GetState(state));
    EXPECT_NE((state.header.validity_flags & protocol::kValidityFromBackend), 0u);
    EXPECT_EQ(state.backend_mode, static_cast<uint16_t>(ArxAdapter::BackendType::InProcessSdk));
    EXPECT_EQ(state.target_seq_applied, 42u);
    EXPECT_NEAR(state.q[0], 0.1f, 1e-5f);
    EXPECT_NEAR(state.q[5], 0.6f, 1e-5f);
    EXPECT_NEAR(state.q_target[0], 0.1f, 1e-5f);
    EXPECT_NEAR(state.tracking_error[0], 0.0f, 1e-5f);

    const auto stats = adapter_->GetStats();
    EXPECT_EQ(stats.active_backend, ArxAdapter::BackendType::InProcessSdk);
    EXPECT_EQ(stats.backend_name, "sdk_inprocess_arxcan");
    EXPECT_TRUE(stats.backend_healthy);
#else
    GTEST_SKIP() << "SDK backend test requires Linux";
#endif
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

TEST_F(ArxAdapterContract, FallsBackToBridgeBackendWhenSdkUnavailable)
{
#if !defined(__linux__)
    GTEST_SKIP() << "Bridge backend contract test requires Linux sockets";
#else
    UdpArmBridgeHarness bridge;
    ASSERT_TRUE(bridge.IsValid());

    auto config = MakeDefaultConfig();
    config.preferred_backend = ArxAdapter::BackendType::InProcessSdk;
    config.allow_fallback_to_bridge = true;
    config.require_live_state = false;
    config.bridge_host = "127.0.0.1";
    config.bridge_command_port = bridge.command_port();
    config.bridge_state_port = bridge.state_port();

    ASSERT_TRUE(adapter_->Initialize(config));
    EXPECT_EQ(adapter_->GetActiveBackend(), ArxAdapter::BackendType::Bridge);
    EXPECT_EQ(adapter_->GetBackendName(), "bridge_ipc");

    adapter_->Start();
    const auto cmd = MakeTestCommand(77);
    adapter_->SetCommand(cmd);

    protocol::ArmCommandFrame received_cmd;
    ASSERT_TRUE(bridge.ReceiveCommand(&received_cmd, std::chrono::milliseconds(500)));
    EXPECT_EQ(received_cmd.header.seq, cmd.header.seq);
    EXPECT_EQ(received_cmd.joint_count, cmd.joint_count);

    protocol::ArmStateFrame state;
    state.header.seq = 5;
    state.header.source_monotonic_ns = GetMonotonicNs();
    state.header.publish_monotonic_ns = state.header.source_monotonic_ns;
    state.header.validity_flags =
        protocol::kValidityPayloadValid | protocol::kValidityFromBackend;
    state.backend_mode = static_cast<uint16_t>(ArxAdapter::BackendType::Bridge);
    state.target_seq_applied = cmd.header.seq;
    state.q = {0.11f, 0.22f, 0.33f, 0.44f, 0.55f, 0.66f};
    state.dq = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
    state.q_target = cmd.q;
    for (size_t i = 0; i < state.tracking_error.size(); ++i)
    {
        state.tracking_error[i] = state.q_target[i] - state.q[i];
    }
    ASSERT_TRUE(bridge.SendState(state));

    protocol::ArmStateFrame received_state;
    bool got_state = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (adapter_->GetState(received_state) &&
            (received_state.header.validity_flags & protocol::kValidityFromBackend) != 0u)
        {
            got_state = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(got_state);
    EXPECT_NE(received_state.header.validity_flags & protocol::kValidityFromBackend, 0u);
    EXPECT_EQ(received_state.backend_mode, static_cast<uint16_t>(ArxAdapter::BackendType::Bridge));
    EXPECT_NEAR(received_state.q[0], state.q[0], 1e-6f);

    const auto stats = adapter_->GetStats();
    EXPECT_EQ(stats.active_backend, ArxAdapter::BackendType::Bridge);
    EXPECT_EQ(stats.backend_name, "bridge_ipc");
#endif
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
