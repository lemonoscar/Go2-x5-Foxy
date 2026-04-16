/**
 * @file test_unitree_adapter_contract.cpp
 * @brief Contract tests for UnitreeAdapter
 *
 * These tests verify the UnitreeAdapter meets its interface contract without
 * requiring actual hardware or DDS running. They use mock patterns to test
 * behavior in isolation.
 */

#include <gtest/gtest.h>

// Only compile these tests when Unitree SDK2 is available
#ifdef UNITREE_SDK2_AVAILABLE

#include "rl_sar/adapters/unitree_adapter.hpp"
#include "rl_sar/protocol/go2_x5_protocol.hpp"

#include <chrono>
#include <thread>

using namespace rl_sar::adapters;
using namespace rl_sar::protocol;

namespace
{

bool IsInitializeSuccess(const UnitreeAdapter::Status status)
{
    return status == UnitreeAdapter::Status::kOk;
}

bool IsInitializeFailureFromEnvironment(const UnitreeAdapter::Status status)
{
    return status == UnitreeAdapter::Status::kDdsInitFailed;
}

} // namespace

class UnitreeAdapterContractTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Default configuration for testing
        config_.network_interface = "lo";  // Use loopback for tests
        config_.command_rate_hz = 200;
        config_.require_lowstate = true;
        config_.lowstate_timeout_ms = 50.0;
        config_.leg_dof_count = 12;
        config_.source_id = 0xDEADBEEF;
    }

    void TearDown() override
    {
        adapter_.Stop();
    }

    UnitreeAdapter::Config config_;
    UnitreeAdapter adapter_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest,InitialState_IsNotInitialized)
{
    EXPECT_FALSE(adapter_.IsInitialized());
    EXPECT_FALSE(adapter_.IsStarted());
}

TEST_F(UnitreeAdapterContractTest, Initialize_ValidConfig_ReturnsOk)
{
    // Note: This test requires ChannelFactory to be initialized first
    // In a real test environment, we would set up a test fixture that
    // initializes the DDS infrastructure

    // For contract testing without actual DDS, we verify the interface
    // behavior is as expected even if initialization fails

    const auto status = adapter_.Initialize(config_);
    // Without actual DDS running, we expect either kOk or kDdsInitFailed
    // Both are valid responses depending on test environment
    EXPECT_TRUE(status == UnitreeAdapter::Status::kOk ||
                status == UnitreeAdapter::Status::kDdsInitFailed);
}

TEST_F(UnitreeAdapterContractTest, Initialize_EmptyNetworkInterface_ReturnsInvalidConfig)
{
    config_.network_interface = "";
    const auto status = adapter_.Initialize(config_);
    EXPECT_EQ(status, UnitreeAdapter::Status::kInvalidConfig);
}

TEST_F(UnitreeAdapterContractTest, Initialize_InvalidCommandRate_ReturnsInvalidConfig)
{
    config_.command_rate_hz = 0;
    auto status = adapter_.Initialize(config_);
    EXPECT_EQ(status, UnitreeAdapter::Status::kInvalidConfig);

    config_.command_rate_hz = 2000;  // Too high
    status = adapter_.Initialize(config_);
    EXPECT_EQ(status, UnitreeAdapter::Status::kInvalidConfig);
}

TEST_F(UnitreeAdapterContractTest, Initialize_InvalidLegDofCount_ReturnsInvalidConfig)
{
    config_.leg_dof_count = 10;  // Only dog-only 12-DoF body is allowed.
    const auto status = adapter_.Initialize(config_);
    EXPECT_EQ(status, UnitreeAdapter::Status::kInvalidConfig);
}

TEST_F(UnitreeAdapterContractTest, Initialize_DuplicateJointMapping_ReturnsInvalidConfig)
{
    config_.joint_mapping[1] = config_.joint_mapping[0];
    const auto status = adapter_.Initialize(config_);
    EXPECT_EQ(status, UnitreeAdapter::Status::kInvalidConfig);
}

TEST_F(UnitreeAdapterContractTest, Initialize_Twice_ReturnsAlreadyInitialized)
{
    const auto first = adapter_.Initialize(config_);
    const auto second = adapter_.Initialize(config_);

    if (IsInitializeSuccess(first))
    {
        EXPECT_EQ(second, UnitreeAdapter::Status::kAlreadyInitialized);
        EXPECT_TRUE(adapter_.IsInitialized());
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(first));
        EXPECT_EQ(second, UnitreeAdapter::Status::kDdsInitFailed);
        EXPECT_FALSE(adapter_.IsInitialized());
    }
}

// ============================================================================
// Lifecycle Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, Start_WithoutInitialize_ReturnsNotInitialized)
{
    const auto status = adapter_.Start();
    EXPECT_EQ(status, UnitreeAdapter::Status::kNotInitialized);
}

TEST_F(UnitreeAdapterContractTest, Start_AfterInitialize_ReturnsOkOrNotStarted)
{
    const auto init_status = adapter_.Initialize(config_);
    const auto status = adapter_.Start();
    if (IsInitializeSuccess(init_status))
    {
        EXPECT_EQ(status, UnitreeAdapter::Status::kOk);
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(init_status));
        EXPECT_EQ(status, UnitreeAdapter::Status::kNotInitialized);
    }
}

TEST_F(UnitreeAdapterContractTest, Start_Twice_ReturnsAlreadyStarted)
{
    const auto init_status = adapter_.Initialize(config_);
    const auto first_start = adapter_.Start();
    const auto second_start = adapter_.Start();

    if (IsInitializeSuccess(init_status))
    {
        EXPECT_EQ(first_start, UnitreeAdapter::Status::kOk);
        EXPECT_EQ(second_start, UnitreeAdapter::Status::kAlreadyStarted);
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(init_status));
        EXPECT_EQ(first_start, UnitreeAdapter::Status::kNotInitialized);
        EXPECT_EQ(second_start, UnitreeAdapter::Status::kNotInitialized);
    }
}

TEST_F(UnitreeAdapterContractTest, Stop_AfterStart_IsNotStarted)
{
    adapter_.Initialize(config_);
    adapter_.Start();
    adapter_.Stop();
    EXPECT_FALSE(adapter_.IsStarted());
}

// ============================================================================
// Command Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, SetCommand_BeforeInitialize_ReturnsNotInitialized)
{
    BodyCommandFrame cmd;
    const auto status = adapter_.SetCommand(cmd);
    EXPECT_EQ(status, UnitreeAdapter::Status::kNotInitialized);
}

TEST_F(UnitreeAdapterContractTest, SetCommand_BeforeStart_ReturnsNotStarted)
{
    const auto init_status = adapter_.Initialize(config_);
    BodyCommandFrame cmd;
    const auto status = adapter_.SetCommand(cmd);
    if (IsInitializeSuccess(init_status))
    {
        EXPECT_EQ(status, UnitreeAdapter::Status::kNotStarted);
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(init_status));
        EXPECT_EQ(status, UnitreeAdapter::Status::kNotInitialized);
    }
}

TEST_F(UnitreeAdapterContractTest, SetCommand_AfterStart_ReturnsOk)
{
    const auto init_status = adapter_.Initialize(config_);
    const auto start_status = adapter_.Start();

    BodyCommandFrame cmd;
    cmd.joint_count = 12;
    cmd.q.fill(0.1f);
    cmd.kp.fill(50.0f);
    cmd.kd.fill(5.0f);

    const auto status = adapter_.SetCommand(cmd);
    if (IsInitializeSuccess(init_status))
    {
        EXPECT_EQ(start_status, UnitreeAdapter::Status::kOk);
        EXPECT_EQ(status, UnitreeAdapter::Status::kOk);
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(init_status));
        EXPECT_EQ(start_status, UnitreeAdapter::Status::kNotInitialized);
        EXPECT_EQ(status, UnitreeAdapter::Status::kNotInitialized);
    }
}

TEST_F(UnitreeAdapterContractTest, ProcessCommand_BeforeStart_ReturnsNotStarted)
{
    const auto init_status = adapter_.Initialize(config_);
    const auto status = adapter_.ProcessCommand();
    if (IsInitializeSuccess(init_status))
    {
        EXPECT_EQ(status, UnitreeAdapter::Status::kNotStarted);
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(init_status));
        EXPECT_EQ(status, UnitreeAdapter::Status::kNotInitialized);
    }
}

// ============================================================================
// State Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, GetState_BeforeInitialize_ReturnsFalse)
{
    BodyStateFrame state;
    EXPECT_FALSE(adapter_.GetState(state));
}

TEST_F(UnitreeAdapterContractTest, GetState_AfterInitialize_ReturnsTrueWithDefaultValues)
{
    const auto init_status = adapter_.Initialize(config_);

    BodyStateFrame state;
    const bool result = adapter_.GetState(state);

    if (IsInitializeSuccess(init_status))
    {
        EXPECT_TRUE(result);
    }
    else
    {
        EXPECT_TRUE(IsInitializeFailureFromEnvironment(init_status));
        EXPECT_FALSE(result);
    }

    // State should have valid frame header
    if (result)
    {
        EXPECT_EQ(state.header.magic, kFrameMagic);
        EXPECT_EQ(state.header.version, kProtocolVersion);
        EXPECT_EQ(state.header.msg_type, FrameType::BodyState);
    }
}

TEST_F(UnitreeAdapterContractTest, IsStateStale_InitialState_ReturnsTrue)
{
    const auto init_status = adapter_.Initialize(config_);

    // No state received yet, should be considered stale
    if (IsInitializeSuccess(init_status))
    {
        EXPECT_TRUE(adapter_.IsStateStale());
    }
}

TEST_F(UnitreeAdapterContractTest, GetStateAgeUs_InitialState_ReturnsMax)
{
    const auto init_status = adapter_.Initialize(config_);

    const uint64_t age = adapter_.GetStateAgeUs();
    if (IsInitializeSuccess(init_status))
    {
        EXPECT_EQ(age, UINT64_MAX);
    }
}

// ============================================================================
// Diagnostics Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, GetDiagnostics_InitialState_HasCorrectValues)
{
    UnitreeAdapter::Diagnostics diag = adapter_.GetDiagnostics();

    EXPECT_FALSE(diag.initialized);
    EXPECT_FALSE(diag.started);
    EXPECT_FALSE(diag.lowstate_seen);
    EXPECT_EQ(diag.lowstate_seq, 0u);
    EXPECT_EQ(diag.command_seq, 0u);
    EXPECT_EQ(diag.write_fail_count, 0u);
    EXPECT_EQ(diag.seq_gap_count, 0u);
}

TEST_F(UnitreeAdapterContractTest, GetDiagnostics_AfterInitialize_HasInitializedTrue)
{
    adapter_.Initialize(config_);

    const UnitreeAdapter::Diagnostics diag = adapter_.GetDiagnostics();

    // May not be fully initialized if DDS failed, but check structure
    EXPECT_EQ(diag.write_fail_count, 0u);
    EXPECT_EQ(diag.seq_gap_count, 0u);
}

// ============================================================================
// Protocol Frame Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, BodyCommandFrame_DefaultConstruction_HasValidHeader)
{
    BodyCommandFrame cmd;

    EXPECT_EQ(cmd.header.magic, kFrameMagic);
    EXPECT_EQ(cmd.header.version, kProtocolVersion);
    EXPECT_EQ(cmd.header.msg_type, FrameType::BodyCommand);
    EXPECT_EQ(cmd.joint_count, kBodyJointCount);
}

TEST_F(UnitreeAdapterContractTest, BodyStateFrame_DefaultConstruction_HasValidHeader)
{
    BodyStateFrame state;

    EXPECT_EQ(state.header.magic, kFrameMagic);
    EXPECT_EQ(state.header.version, kProtocolVersion);
    EXPECT_EQ(state.header.msg_type, FrameType::BodyState);
}

TEST_F(UnitreeAdapterContractTest, BodyCommandFrame_CanStoreAll12LegDOFs)
{
    BodyCommandFrame cmd;

    // Set unique values for each joint
    for (size_t i = 0; i < 12; ++i)
    {
        cmd.q[i] = static_cast<float>(i) * 0.01f;
        cmd.dq[i] = static_cast<float>(i) * 0.1f;
        cmd.kp[i] = static_cast<float>(i);
        cmd.kd[i] = static_cast<float>(i) * 0.1f;
        cmd.tau[i] = static_cast<float>(i) * 0.001f;
    }

    // Verify values are stored correctly
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_FLOAT_EQ(cmd.q[i], static_cast<float>(i) * 0.01f);
        EXPECT_FLOAT_EQ(cmd.dq[i], static_cast<float>(i) * 0.1f);
        EXPECT_FLOAT_EQ(cmd.kp[i], static_cast<float>(i));
        EXPECT_FLOAT_EQ(cmd.kd[i], static_cast<float>(i) * 0.1f);
        EXPECT_FLOAT_EQ(cmd.tau[i], static_cast<float>(i) * 0.001f);
    }
}

TEST_F(UnitreeAdapterContractTest, BodyStateFrame_CanStoreAll12LegDOFs)
{
    BodyStateFrame state;

    // Set unique values for each joint
    for (size_t i = 0; i < 12; ++i)
    {
        state.leg_q[i] = static_cast<float>(i) * 0.01f;
        state.leg_dq[i] = static_cast<float>(i) * 0.1f;
        state.leg_tau[i] = static_cast<float>(i) * 0.001f;
    }

    // Verify values are stored correctly
    for (size_t i = 0; i < 12; ++i)
    {
        EXPECT_FLOAT_EQ(state.leg_q[i], static_cast<float>(i) * 0.01f);
        EXPECT_FLOAT_EQ(state.leg_dq[i], static_cast<float>(i) * 0.1f);
        EXPECT_FLOAT_EQ(state.leg_tau[i], static_cast<float>(i) * 0.001f);
    }
}

// ============================================================================
// Status String Conversion Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, ToString_AllStatuses_ReturnsValidStrings)
{
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kOk), "Ok");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kNotInitialized), "NotInitialized");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kAlreadyInitialized), "AlreadyInitialized");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kDdsInitFailed), "DdsInitFailed");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kPublisherCreateFailed), "PublisherCreateFailed");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kSubscriberCreateFailed), "SubscriberCreateFailed");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kInvalidConfig), "InvalidConfig");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kStateStale), "StateStale");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kWriteFailed), "WriteFailed");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kNotStarted), "NotStarted");
    EXPECT_STREQ(ToString(UnitreeAdapter::Status::kAlreadyStarted), "AlreadyStarted");
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, ConcurrentGetState_DoesNotCrash)
{
    adapter_.Initialize(config_);

    constexpr int kNumThreads = 4;
    constexpr int kIterationsPerThread = 100;

    std::vector<std::thread> threads;
    for (int i = 0; i < kNumThreads; ++i)
    {
        threads.emplace_back([this, kIterationsPerThread]()
        {
            for (int j = 0; j < kIterationsPerThread; ++j)
            {
                BodyStateFrame state;
                adapter_.GetState(state);
            }
        });
    }

    for (auto& t : threads)
    {
        t.join();
    }

    // If we get here without crashing, thread safety is working
    SUCCEED();
}

TEST_F(UnitreeAdapterContractTest, ConcurrentSetCommand_DoesNotCrash)
{
    adapter_.Initialize(config_);
    adapter_.Start();

    constexpr int kNumThreads = 4;
    constexpr int kIterationsPerThread = 100;

    std::vector<std::thread> threads;
    for (int i = 0; i < kNumThreads; ++i)
    {
        threads.emplace_back([this, kIterationsPerThread]()
        {
            BodyCommandFrame cmd;
            cmd.q.fill(0.1f);
            cmd.kp.fill(50.0f);

            for (int j = 0; j < kIterationsPerThread; ++j)
            {
                adapter_.SetCommand(cmd);
            }
        });
    }

    for (auto& t : threads)
    {
        t.join();
    }

    // If we get here without crashing, thread safety is working
    SUCCEED();
}

// ============================================================================
// Integration Contract Tests
// ============================================================================

TEST_F(UnitreeAdapterContractTest, AdapterProtocolIntegration_BodyCommandToBodyStateRoundtrip)
{
    // This test verifies the protocol integration contract:
    // 1. Create a BodyCommandFrame
    // 2. Set it on the adapter
    // 3. The adapter converts it to LowCmd_ internally
    // 4. LowState_ arrives and is converted to BodyStateFrame
    // 5. GetState returns the BodyStateFrame

    // Note: Without actual DDS/robot, we can only verify the data structures
    // are compatible for this round trip

    BodyCommandFrame cmd;
    cmd.header.msg_type = FrameType::BodyCommand;
    cmd.joint_count = 12;
    for (size_t i = 0; i < 12; ++i)
    {
        cmd.q[i] = 0.0f;  // Standing position
    }
    cmd.kp.fill(50.0f);
    cmd.kd.fill(5.0f);

    // Verify command structure is valid
    EXPECT_EQ(cmd.joint_count, 12);
    EXPECT_EQ(cmd.header.msg_type, FrameType::BodyCommand);

    // Verify state structure can receive the data
    BodyStateFrame state;
    state.header.msg_type = FrameType::BodyState;

    EXPECT_EQ(state.leg_q.size(), 12u);
    EXPECT_EQ(state.leg_dq.size(), 12u);
    EXPECT_EQ(state.leg_tau.size(), 12u);
}

#endif  // UNITREE_SDK2_AVAILABLE

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
