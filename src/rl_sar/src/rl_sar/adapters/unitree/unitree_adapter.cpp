#include "rl_sar/adapters/unitree_adapter.hpp"

// Unitree SDK2 headers - these must be included after our header
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace unitree::robot;

// Topic constants for Go2 DDS communication
constexpr const char* TOPIC_LOWCMD = "rt/lowcmd";
constexpr const char* TOPIC_LOWSTATE = "rt/lowstate";

// Unitree low-level command constants
constexpr double kPosStopF = (2.146E+9f);
constexpr double kVelStopF = (16000.0f);
constexpr uint8_t kLowCmdHeader0 = 0xFE;
constexpr uint8_t kLowCmdHeader1 = 0xEF;
constexpr uint8_t kLowCmdLevelFlag = 0xFF;
constexpr uint8_t kMotorModePseudoContinuous = 0x01;
constexpr uint8_t kMotorModeDisabled = 0x00;

// Unitree Go2 has 20 motors total (12 leg + 6 arm + 2 waist)
constexpr int kUnitreeMotorCount = 20;

namespace rl_sar::adapters
{

// ============================================================================
// PIMPL Implementation for DDS types (header isolation)
// ============================================================================

struct UnitreeAdapter::LowCmdPublisher
{
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> publisher;
};

struct UnitreeAdapter::LowStateSubscriber
{
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> subscriber;
};

// ============================================================================
// Construction/Destruction
// ============================================================================

UnitreeAdapter::UnitreeAdapter()
    : state_stamp_{}
    , last_write_stamp_{}
    , source_monotonic_ns_(0)
{
    // Initialize pending command with safe defaults
    pending_command_.header.msg_type = protocol::FrameType::BodyCommand;
    pending_command_.header.version = protocol::kProtocolVersion;
    pending_command_.header.magic = protocol::kFrameMagic;
    pending_command_.joint_count = protocol::kDogJointCount;
    pending_command_.q.fill(0.0f);
    pending_command_.dq.fill(0.0f);
    pending_command_.kp.fill(0.0f);
    pending_command_.kd.fill(0.0f);
    pending_command_.tau.fill(0.0f);

    // Initialize latest state with safe defaults
    latest_state_.header.msg_type = protocol::FrameType::BodyState;
    latest_state_.header.version = protocol::kProtocolVersion;
    latest_state_.header.magic = protocol::kFrameMagic;
    latest_state_.imu_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    latest_state_.imu_gyro.fill(0.0f);
    latest_state_.imu_acc.fill(0.0f);
    latest_state_.leg_q.fill(0.0f);
    latest_state_.leg_dq.fill(0.0f);
    latest_state_.leg_tau.fill(0.0f);
    latest_state_.base_lin_vel.fill(0.0f);
    latest_state_.base_ang_vel.fill(0.0f);
    latest_state_.projected_gravity.fill(0.0f);
    latest_state_.lowstate_age_us = 0;
    latest_state_.dds_ok = 0;
}

UnitreeAdapter::~UnitreeAdapter()
{
    Stop();
}

// ============================================================================
// Initialization and Lifecycle
// ============================================================================

UnitreeAdapter::Status UnitreeAdapter::Initialize(const Config& config)
{
    if (initialized_.load(std::memory_order_acquire))
    {
        return Status::kAlreadyInitialized;
    }

    // Validate configuration
    if (config.network_interface.empty())
    {
        return Status::kInvalidConfig;
    }
    if (config.command_rate_hz <= 0 || config.command_rate_hz > 1000)
    {
        return Status::kInvalidConfig;
    }
    if (config.leg_dof_count != 12 && config.leg_dof_count != 20)
    {
        return Status::kInvalidConfig;
    }

    config_ = config;

    try
    {
        // Initialize Unitree SDK2 ChannelFactory
        // Note: ChannelFactory::Instance()->Init() should be called once per process
        // We assume it's initialized by the main application
        // If not initialized, we attempt to initialize it here
        ChannelFactory::Instance()->Init(0, config_.network_interface.c_str());

        // Create lowcmd publisher
        lowcmd_publisher_ = std::make_unique<LowCmdPublisher>();
        lowcmd_publisher_->publisher.reset(
            new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher_->publisher->InitChannel();

        // Create lowstate subscriber
        lowstate_subscriber_ = std::make_unique<LowStateSubscriber>();
        lowstate_subscriber_->subscriber.reset(
            new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

        // Bind the callback with queue size of 1 (always get latest)
        lowstate_subscriber_->subscriber->InitChannel(
            [this](const void* msg) { this->LowStateMessageHandler(msg); },
            1);

        // Allocate lowcmd buffer
        lowcmd_buffer_ = std::make_unique<unitree_go::msg::dds_::LowCmd_>();
        InitializeLowCmd();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[UnitreeAdapter] Initialization failed: " << e.what() << std::endl;
        lowcmd_publisher_.reset();
        lowstate_subscriber_.reset();
        lowcmd_buffer_.reset();
        return Status::kDdsInitFailed;
    }

    // Initialize source timestamp
    source_monotonic_ns_ = GetMonotonicNs();

    initialized_.store(true, std::memory_order_release);
    return Status::kOk;
}

UnitreeAdapter::Status UnitreeAdapter::Start()
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return Status::kNotInitialized;
    }

    if (started_.load(std::memory_order_acquire))
    {
        return Status::kAlreadyStarted;
    }

    // Reset diagnostic counters
    write_fail_count_.store(0, std::memory_order_relaxed);
    seq_gap_count_.store(0, std::memory_order_relaxed);
    lowstate_seen_.store(false, std::memory_order_release);
    lowstate_seq_.store(0, std::memory_order_release);
    command_seq_.store(0, std::memory_order_release);

    started_.store(true, std::memory_order_release);
    return Status::kOk;
}

void UnitreeAdapter::Stop()
{
    if (!started_.load(std::memory_order_acquire))
    {
        return;
    }

    started_.store(false, std::memory_order_release);

    // Send zero commands before shutdown to ensure safe stop
    if (lowcmd_publisher_ && lowcmd_buffer_)
    {
        InitializeLowCmd();
        try
        {
            if (lowcmd_publisher_->publisher)
            {
                lowcmd_publisher_->publisher->Write(*lowcmd_buffer_);
            }
        }
        catch (...)
        {
            // Ignore errors during shutdown
        }
    }

    // Clean up DDS resources
    lowcmd_publisher_.reset();
    lowstate_subscriber_.reset();
    lowcmd_buffer_.reset();
}

// ============================================================================
// Command Processing
// ============================================================================

UnitreeAdapter::Status UnitreeAdapter::SetCommand(const protocol::BodyCommandFrame& cmd)
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return Status::kNotInitialized;
    }

    if (!started_.load(std::memory_order_acquire))
    {
        return Status::kNotStarted;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_command_ = cmd;
    return Status::kOk;
}

UnitreeAdapter::Status UnitreeAdapter::ProcessCommand()
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return Status::kNotInitialized;
    }

    if (!started_.load(std::memory_order_acquire))
    {
        return Status::kNotStarted;
    }

    // Get pending command
    protocol::BodyCommandFrame cmd;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd = pending_command_;
    }

    // Update command frame metadata
    cmd.header.seq = command_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
    cmd.header.source_monotonic_ns = GetMonotonicNs();
    cmd.header.publish_monotonic_ns = cmd.header.source_monotonic_ns;
    cmd.header.source_id = config_.source_id;

    // Convert to LowCmd_ and write
    ConvertBodyCommandToLowCmd(cmd);

    try
    {
        if (lowcmd_publisher_ && lowcmd_publisher_->publisher)
        {
            lowcmd_publisher_->publisher->Write(*lowcmd_buffer_);
            last_write_stamp_ = std::chrono::steady_clock::now();
            return Status::kOk;
        }
        else
        {
            return Status::kWriteFailed;
        }
    }
    catch (const std::exception& e)
    {
        write_fail_count_.fetch_add(1, std::memory_order_relaxed);
        std::cerr << "[UnitreeAdapter] DDS write failed: " << e.what() << std::endl;
        return Status::kWriteFailed;
    }
}

// ============================================================================
// State Retrieval
// ============================================================================

bool UnitreeAdapter::GetState(protocol::BodyStateFrame& state)
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;

    // Update lowstate age based on current time
    const auto now = std::chrono::steady_clock::now();
    if (state_stamp_.time_since_epoch().count() > 0)
    {
        const auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - state_stamp_);
        state.lowstate_age_us = static_cast<uint32_t>(age.count());
    }
    else
    {
        state.lowstate_age_us = UINT32_MAX;
    }

    return true;
}

bool UnitreeAdapter::IsStateStale() const
{
    const uint64_t age_us = GetStateAgeUs();
    const uint64_t timeout_us = static_cast<uint64_t>(config_.lowstate_timeout_ms * 1000.0);
    return age_us > timeout_us;
}

uint64_t UnitreeAdapter::GetStateAgeUs() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_stamp_.time_since_epoch().count() == 0)
    {
        return UINT64_MAX;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - state_stamp_);
    return static_cast<uint64_t>(age.count());
}

UnitreeAdapter::Diagnostics UnitreeAdapter::GetDiagnostics() const
{
    Diagnostics diag;
    diag.initialized = initialized_.load(std::memory_order_acquire);
    diag.started = started_.load(std::memory_order_acquire);
    diag.lowstate_seen = lowstate_seen_.load(std::memory_order_acquire);
    diag.dds_write_ok = write_fail_count_.load(std::memory_order_relaxed) == 0;
    diag.lowstate_seq = lowstate_seq_.load(std::memory_order_acquire);
    diag.command_seq = command_seq_.load(std::memory_order_acquire);
    diag.lowstate_age_us = const_cast<UnitreeAdapter*>(this)->GetStateAgeUs();
    diag.write_fail_count = write_fail_count_.load(std::memory_order_relaxed);
    diag.seq_gap_count = seq_gap_count_.load(std::memory_order_relaxed);

    // Calculate last write age
    if (last_write_stamp_.time_since_epoch().count() > 0)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - last_write_stamp_);
        diag.last_write_age_us = static_cast<uint64_t>(age.count());
    }
    else
    {
        diag.last_write_age_us = UINT64_MAX;
    }

    return diag;
}

// ============================================================================
// Internal DDS Callback
// ============================================================================

void UnitreeAdapter::LowStateMessageHandler(const void* message)
{
    const auto* msg = static_cast<const unitree_go::msg::dds_::LowState_*>(message);
    if (!msg)
    {
        return;
    }

    const uint64_t now_ns = GetMonotonicNs();

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Copy IMU data
    for (size_t i = 0; i < latest_state_.imu_quat.size(); ++i)
    {
        latest_state_.imu_quat[i] = msg->imu_state().quaternion()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < latest_state_.imu_gyro.size(); ++i)
    {
        latest_state_.imu_gyro[i] = msg->imu_state().gyroscope()[static_cast<int>(i)];
    }
    for (size_t i = 0; i < latest_state_.imu_acc.size(); ++i)
    {
        latest_state_.imu_acc[i] = msg->imu_state().accelerometer()[static_cast<int>(i)];
    }

    // Copy motor state (we only care about the first 12 for leg DOFs)
    const int leg_count = std::min(static_cast<int>(config_.leg_dof_count), kUnitreeMotorCount);
    for (int i = 0; i < leg_count; ++i)
    {
        const auto& motor_state = msg->motor_state()[i];
        latest_state_.leg_q[static_cast<size_t>(i)] = motor_state.q();
        latest_state_.leg_dq[static_cast<size_t>(i)] = motor_state.dq();
        latest_state_.leg_tau[static_cast<size_t>(i)] = motor_state.tau_est();
    }

    // Update frame metadata
    latest_state_.header.source_monotonic_ns = now_ns;
    latest_state_.header.publish_monotonic_ns = now_ns;
    latest_state_.header.source_id = config_.source_id;

    // Mark DDS as OK
    latest_state_.dds_ok = 1;

    // Update state stamp
    state_stamp_ = std::chrono::steady_clock::now();

    // Mark lowstate as seen and increment sequence
    if (!lowstate_seen_.load(std::memory_order_acquire))
    {
        lowstate_seen_.store(true, std::memory_order_release);
    }

    // Increment lowstate sequence
    IncrementLowstateSeq();
}

// ============================================================================
// Internal Helpers
// ============================================================================

void UnitreeAdapter::InitializeLowCmd()
{
    if (!lowcmd_buffer_)
    {
        return;
    }

    auto& cmd = *lowcmd_buffer_;
    cmd.head()[0] = kLowCmdHeader0;
    cmd.head()[1] = kLowCmdHeader1;
    cmd.level_flag() = kLowCmdLevelFlag;
    cmd.gpio() = 0;

    for (int i = 0; i < kUnitreeMotorCount; ++i)
    {
        cmd.motor_cmd()[i].mode() = kMotorModePseudoContinuous;
        cmd.motor_cmd()[i].q() = kPosStopF;
        cmd.motor_cmd()[i].kp() = 0.0f;
        cmd.motor_cmd()[i].dq() = kVelStopF;
        cmd.motor_cmd()[i].kd() = 0.0f;
        cmd.motor_cmd()[i].tau() = 0.0f;
    }
}

void UnitreeAdapter::ConvertBodyCommandToLowCmd(const protocol::BodyCommandFrame& body_cmd)
{
    if (!lowcmd_buffer_)
    {
        return;
    }

    auto& cmd = *lowcmd_buffer_;

    // Keep header and GPIO set from initialization
    // motor commands are updated based on body_cmd

    // Map the 12 leg DOFs to the first 12 motors in Unitree's motor array
    const int leg_count = std::min(static_cast<int>(body_cmd.joint_count),
                                   static_cast<int>(config_.leg_dof_count));
    const int motor_count = std::min(leg_count, kUnitreeMotorCount);

    for (int i = 0; i < motor_count; ++i)
    {
        auto& motor_cmd = cmd.motor_cmd()[i];

        // Check if we have valid command data
        const bool has_valid_cmd =
            std::isfinite(body_cmd.q[static_cast<size_t>(i)]) ||
            std::isfinite(body_cmd.tau[static_cast<size_t>(i)]);

        if (has_valid_cmd)
        {
            motor_cmd.mode() = kMotorModePseudoContinuous;

            // Use feedforward torque if position is not finite
            if (std::isfinite(body_cmd.q[static_cast<size_t>(i)]))
            {
                motor_cmd.q() = body_cmd.q[static_cast<size_t>(i)];
                motor_cmd.dq() = std::isfinite(body_cmd.dq[static_cast<size_t>(i)])
                    ? body_cmd.dq[static_cast<size_t>(i)]
                    : 0.0f;
                motor_cmd.kp() = std::isfinite(body_cmd.kp[static_cast<size_t>(i)])
                    ? body_cmd.kp[static_cast<size_t>(i)]
                    : 0.0f;
                motor_cmd.kd() = std::isfinite(body_cmd.kd[static_cast<size_t>(i)])
                    ? body_cmd.kd[static_cast<size_t>(i)]
                    : 0.0f;
                motor_cmd.tau() = std::isfinite(body_cmd.tau[static_cast<size_t>(i)])
                    ? body_cmd.tau[static_cast<size_t>(i)]
                    : 0.0f;
            }
            else
            {
                // Torque-only mode
                motor_cmd.q() = kPosStopF;
                motor_cmd.dq() = kVelStopF;
                motor_cmd.kp() = 0.0f;
                motor_cmd.kd() = 0.0f;
                motor_cmd.tau() = std::isfinite(body_cmd.tau[static_cast<size_t>(i)])
                    ? body_cmd.tau[static_cast<size_t>(i)]
                    : 0.0f;
            }
        }
        else
        {
            // No valid command - set to safe defaults
            motor_cmd.mode() = kMotorModeDisabled;
            motor_cmd.q() = kPosStopF;
            motor_cmd.dq() = kVelStopF;
            motor_cmd.kp() = 0.0f;
            motor_cmd.kd() = 0.0f;
            motor_cmd.tau() = 0.0f;
        }
    }

    // For any additional motors beyond leg_count, set to disabled
    for (int i = motor_count; i < kUnitreeMotorCount; ++i)
    {
        cmd.motor_cmd()[i].mode() = kMotorModeDisabled;
        cmd.motor_cmd()[i].q() = kPosStopF;
        cmd.motor_cmd()[i].dq() = kVelStopF;
        cmd.motor_cmd()[i].kp() = 0.0f;
        cmd.motor_cmd()[i].kd() = 0.0f;
        cmd.motor_cmd()[i].tau() = 0.0f;
    }

    // Calculate CRC32 (Unitree requires this for valid commands)
    // The CRC is calculated over the entire LowCmd_ structure
    // For now, we set it to 0 as the SDK may handle this
    cmd.crc() = 0;
}

void UnitreeAdapter::IncrementLowstateSeq()
{
    lowstate_seq_.fetch_add(1, std::memory_order_release);
}

uint64_t UnitreeAdapter::GetMonotonicNs()
{
    const auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

// ============================================================================
// Utility Functions
// ============================================================================

const char* ToString(UnitreeAdapter::Status status)
{
    switch (status)
    {
        case UnitreeAdapter::Status::kOk:                     return "Ok";
        case UnitreeAdapter::Status::kNotInitialized:         return "NotInitialized";
        case UnitreeAdapter::Status::kAlreadyInitialized:     return "AlreadyInitialized";
        case UnitreeAdapter::Status::kDdsInitFailed:          return "DdsInitFailed";
        case UnitreeAdapter::Status::kPublisherCreateFailed:  return "PublisherCreateFailed";
        case UnitreeAdapter::Status::kSubscriberCreateFailed: return "SubscriberCreateFailed";
        case UnitreeAdapter::Status::kInvalidConfig:          return "InvalidConfig";
        case UnitreeAdapter::Status::kStateStale:             return "StateStale";
        case UnitreeAdapter::Status::kWriteFailed:            return "WriteFailed";
        case UnitreeAdapter::Status::kNotStarted:             return "NotStarted";
        case UnitreeAdapter::Status::kAlreadyStarted:         return "AlreadyStarted";
        default:                                              return "Unknown";
    }
}

}  // namespace rl_sar::adapters
