#ifndef RL_SAR_ADAPTERS_UNITREE_ADAPTER_HPP
#define RL_SAR_ADAPTERS_UNITREE_ADAPTER_HPP

#include "rl_sar/protocol/go2_x5_protocol.hpp"
#include "rl_sar/state_estimation/velocity_estimator.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// Forward declarations for Unitree SDK2 types
namespace unitree::robot
{
template<typename T>
class ChannelPublisher;
template<typename T>
class ChannelSubscriber;
}

namespace unitree_go::msg::dds_
{
struct LowCmd_;
struct LowState_;
}

namespace rl_sar::adapters
{

/**
 * @brief Adapter for Unitree Go2 robot DDS communication
 *
 * This adapter isolates all DDS (Data Distribution Service) details from the main
 * control logic, providing a clean interface using typed frames (BodyStateFrame and
 * BodyCommandFrame).
 *
 * Key responsibilities:
 * - Unitree SDK2 initialization and lifecycle management
 * - Reading from rt/lowstate DDS topic and converting to BodyStateFrame
 * - Writing to rt/lowcmd DDS topic from BodyCommandFrame
 * - Thread-safe state management
 * - Health monitoring with timeout detection
 *
 * The adapter runs at 200Hz command rate and provides state freshness checking
 * for supervision and safety systems.
 */
class UnitreeAdapter
{
public:
    /**
     * @brief Configuration for the UnitreeAdapter
     */
    struct Config
    {
        /// Network interface for DDS communication
        std::string network_interface = "eth0";

        /// Command publishing rate in Hz (default: 200Hz for Go2)
        int command_rate_hz = 200;

        /// Whether lowstate reception is required for operation
        bool require_lowstate = true;

        /// Timeout threshold for lowstate staleness detection in milliseconds
        double lowstate_timeout_ms = 50.0;

        /// Number of leg DOFs (12 for Go2)
        uint16_t leg_dof_count = 12;

        /// Mapping from typed body joint slot [0, 11] to Unitree motor slot.
        std::array<int, protocol::kDogJointCount> joint_mapping{
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}
        };

        /// Source ID for frame headers
        uint32_t source_id = 0;

        /// Whether ChannelFactory::Init should be performed by the adapter.
        bool initialize_channel_factory = true;

        /// Whether to enable velocity estimation
        bool enable_velocity_estimation = true;

        /// Velocity estimator configuration
        state_estimation::VelocityEstimator::Config velocity_estimator_config;
    };

    /**
     * @brief Status codes for adapter operations
     */
    enum class Status : uint8_t
    {
        kOk = 0,
        kNotInitialized = 1,
        kAlreadyInitialized = 2,
        kDdsInitFailed = 3,
        kPublisherCreateFailed = 4,
        kSubscriberCreateFailed = 5,
        kInvalidConfig = 6,
        kStateStale = 7,
        kWriteFailed = 8,
        kNotStarted = 9,
        kAlreadyStarted = 10,
    };

    /**
     * @brief Diagnostic information about the adapter state
     */
    struct Diagnostics
    {
        bool initialized = false;
        bool started = false;
        bool lowstate_seen = false;
        bool dds_write_ok = true;
        uint64_t lowstate_seq = 0;
        uint64_t command_seq = 0;
        uint64_t lowstate_age_us = 0;
        uint64_t last_write_age_us = 0;
        uint32_t write_fail_count = 0;
        uint32_t seq_gap_count = 0;
    };

    UnitreeAdapter();
    ~UnitreeAdapter();

    // Disable copy and move to ensure safe DDS resource management
    UnitreeAdapter(const UnitreeAdapter&) = delete;
    UnitreeAdapter& operator=(const UnitreeAdapter&) = delete;
    UnitreeAdapter(UnitreeAdapter&&) = delete;
    UnitreeAdapter& operator=(UnitreeAdapter&&) = delete;

    /**
     * @brief Initialize the Unitree adapter with DDS channels
     *
     * This must be called before any other operations. It initializes the
     * Unitree SDK2 ChannelFactory and creates DDS publisher/subscriber.
     *
     * @param config Configuration parameters
     * @return Status indicating success or failure reason
     */
    Status Initialize(const Config& config);

    /**
     * @brief Start the adapter (begins DDS communication)
     *
     * After calling Start(), the adapter will begin receiving lowstate
     * messages and accepting command writes.
     *
     * @return Status indicating success or failure reason
     */
    Status Start();

    /**
     * @brief Stop the adapter and clean up DDS resources
     */
    void Stop();

    /**
     * @brief Set a new body command to be sent to the robot
     *
     * This copies the command into internal state. The actual DDS write
     * happens when ProcessCommand() is called.
     *
     * Thread-safe: can be called from any thread.
     *
     * @param cmd Body command frame to send
     * @return Status indicating success or failure reason
     */
    Status SetCommand(const protocol::BodyCommandFrame& cmd);

    /**
     * @brief Get the latest body state from the robot
     *
     * Thread-safe: can be called from any thread.
     *
     * @param state Output parameter for the body state
     * @return true if state was retrieved (even if stale), false if not initialized
     */
    bool GetState(protocol::BodyStateFrame& state);

    /**
     * @brief Process and send the current command via DDS
     *
     * This should be called at the configured command rate (typically 200Hz).
     * It converts the BodyCommandFrame to LowCmd_ and writes to the DDS topic.
     *
     * @return Status indicating success or failure reason
     */
    Status ProcessCommand();

    /**
     * @brief Check if the current state is stale (older than timeout threshold)
     *
     * @return true if state is stale or never received, false if fresh
     */
    bool IsStateStale() const;

    /**
     * @brief Get the age of the current state in microseconds
     *
     * @return Age in microseconds, or UINT64_MAX if state never received
     */
    uint64_t GetStateAgeUs() const;

    /**
     * @brief Get the current diagnostics
     *
     * @return Current diagnostic information
     */
    Diagnostics GetDiagnostics() const;

    /**
     * @brief Check if the adapter is initialized
     */
    bool IsInitialized() const { return initialized_.load(std::memory_order_acquire); }

    /**
     * @brief Check if the adapter is started
     */
    bool IsStarted() const { return started_.load(std::memory_order_acquire); }

    /**
     * @brief Get the last lowstate sequence number
     */
    uint64_t GetLowstateSeq() const { return lowstate_seq_.load(std::memory_order_acquire); }

private:
    // DDS callback handler for lowstate messages
    void LowStateMessageHandler(const void* message);

    // Internal command conversion helpers
    void ConvertBodyCommandToLowCmd(const protocol::BodyCommandFrame& body_cmd);
    void InitializeLowCmd();
    int ResolveMotorIndexForBodyJoint(int body_joint_index) const;
    static uint32_t ComputeLowCmdCrc(const unitree_go::msg::dds_::LowCmd_& cmd);

    // Sequence tracking
    uint64_t IncrementLowstateSeq();

    // Monotonic time utility
    static uint64_t GetMonotonicNs();

    // Configuration
    Config config_;

    // DDS channels (opaque pointers to hide implementation)
    struct LowCmdPublisher;
    struct LowStateSubscriber;
    std::unique_ptr<LowCmdPublisher> lowcmd_publisher_;
    std::unique_ptr<LowStateSubscriber> lowstate_subscriber_;

    // Cached command data (for safe cross-thread access)
    protocol::BodyCommandFrame pending_command_;
    std::mutex command_mutex_;

    // Cached state data (for safe cross-thread access)
    protocol::BodyStateFrame latest_state_;
    std::chrono::steady_clock::time_point state_stamp_;
    mutable std::mutex state_mutex_;

    // LowCmd_ buffer for DDS writing
    std::unique_ptr<unitree_go::msg::dds_::LowCmd_> lowcmd_buffer_;

    // Atomic state tracking
    std::atomic<bool> initialized_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> lowstate_seen_{false};
    std::atomic<uint64_t> lowstate_seq_{0};
    std::atomic<uint64_t> command_seq_{0};
    std::atomic<uint64_t> previous_lowstate_msg_seq_{0};

    // Diagnostics
    std::atomic<uint32_t> write_fail_count_{0};
    std::atomic<uint32_t> seq_gap_count_{0};
    std::chrono::steady_clock::time_point last_write_stamp_;
    mutable std::mutex diagnostics_mutex_;

    // Source monotonic timestamp for seq tracking
    uint64_t source_monotonic_ns_;

    // Velocity estimation
    std::unique_ptr<state_estimation::VelocityEstimator> velocity_estimator_;
    std::array<float, 4> foot_force_;  // 4 foot contact forces in [FR, FL, RR, RL]
    std::array<float, 12> leg_position_buffer_;  // Joint positions in project order
    std::array<float, 12> leg_velocity_buffer_;  // Joint velocities in project order
    bool velocity_estimator_ready_ = false;
};

/**
 * @brief Convert a UnitreeAdapter Status to a human-readable string
 */
const char* ToString(UnitreeAdapter::Status status);

}  // namespace rl_sar::adapters

#endif  // RL_SAR_ADAPTERS_UNITREE_ADAPTER_HPP
