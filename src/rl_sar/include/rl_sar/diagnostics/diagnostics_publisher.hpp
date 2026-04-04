#ifndef RL_SAR_DIAGNOSTICS_DIAGNOSTICS_PUBLISHER_HPP
#define RL_SAR_DIAGNOSTICS_DIAGNOSTICS_PUBLISHER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

// ROS2 support (conditional compilation)
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif  // USE_ROS2

namespace rl_sar::diagnostics
{

/**
 * @brief Comprehensive diagnostic metrics for sim2real validation
 *
 * This structure captures all key metrics required by the validation checklist
 * for monitoring system health and detecting simulation-to-real gaps.
 */
struct DiagnosticMetrics
{
    // Policy metrics
    uint64_t policy_latency_us = 0;         ///< Time from observation to action output (microseconds)
    double policy_frequency_hz = 0.0;       ///< Measured policy execution frequency

    // Coordinator metrics
    uint64_t coordinator_jitter_us = 0;     ///< Deviation from expected coordinator timing (microseconds)
    double coordinator_frequency_hz = 0.0;  ///< Measured coordinator loop frequency

    // State freshness
    uint64_t body_state_age_us = 0;         ///< Age of body state from source (microseconds)
    uint64_t arm_state_age_us = 0;          ///< Age of arm state from backend (microseconds)

    // Arm tracking
    double arm_tracking_error_norm = 0.0;   ///< L2 norm of joint tracking error
    bool arm_tracking_healthy = true;       ///< Overall arm tracking health flag

    // Drift metrics (for zero-velocity validation)
    double xy_drift_error = 0.0;            ///< Euclidean XY drift from reference (meters)
    double yaw_drift_error = 0.0;           ///< Absolute yaw drift from reference (radians)

    // Safety indicators
    int clip_count = 0;                     ///< Number of safety clip events
    int seq_gap_count = 0;                  ///< Number of sequence gaps detected

    // Mode information
    std::string current_mode;               ///< Current operational mode string
    uint32_t mode_transition_count = 0;     ///< Total number of mode transitions

    // Timestamp
    uint64_t timestamp_ns = 0;              ///< Monotonic timestamp of this metrics snapshot

    /**
     * @brief Reset all metrics to default values
     */
    void Reset()
    {
        policy_latency_us = 0;
        policy_frequency_hz = 0.0;
        coordinator_jitter_us = 0;
        coordinator_frequency_hz = 0.0;
        body_state_age_us = 0;
        arm_state_age_us = 0;
        arm_tracking_error_norm = 0.0;
        arm_tracking_healthy = true;
        xy_drift_error = 0.0;
        yaw_drift_error = 0.0;
        clip_count = 0;
        seq_gap_count = 0;
        current_mode.clear();
        mode_transition_count = 0;
        timestamp_ns = 0;
    }

    /**
     * @brief Check if metrics indicate a degraded state
     * @return true if any critical metric is outside healthy bounds
     */
    bool IsDegraded() const
    {
        constexpr uint64_t kMaxStateAgeUs = 500000;        // 500ms
        constexpr uint64_t kMaxPolicyLatencyUs = 50000;    // 50ms
        constexpr double kMaxTrackingError = 0.5;          // radians

        return (body_state_age_us > kMaxStateAgeUs) ||
               (arm_state_age_us > kMaxStateAgeUs) ||
               (policy_latency_us > kMaxPolicyLatencyUs) ||
               !arm_tracking_healthy ||
               (arm_tracking_error_norm > kMaxTrackingError);
    }

    /**
     * @brief Get a human-readable status string
     * @return Status description string
     */
    std::string GetStatusString() const
    {
        if (IsDegraded()) {
            return "DEGRADED";
        }
        return "HEALTHY";
    }
};

/**
 * @brief Real-time diagnostics publisher for sim2real validation
 *
 * This component publishes key metrics at high frequency (50-100Hz) for
 * external monitoring and validation. It supports both ROS2 topic publishing
 * and internal-only modes for standalone operation.
 *
 * Key features:
 * - Thread-safe metrics update from multiple sources
 * - Configurable publish rate (default 50Hz)
 * - ROS2 topic publishing with configurable topic name
 * - Internal metrics buffer for snapshot access
 * - Graceful degradation when ROS2 unavailable
 * - Support for both ROS2 Foxy and standalone modes
 *
 * Usage example:
 * @code
 * DiagnosticsPublisher publisher;
 * DiagnosticsPublisher::Config config;
 * config.ros2_enabled = true;
 * config.publish_rate_hz = 50;
 * config.diagnostics_topic = "/go2_x5/diagnostics";
 *
 * if (publisher.Initialize(config)) {
 *     publisher.Start();
 *
 *     while (running) {
 *         DiagnosticMetrics metrics;
 *         // ... populate metrics ...
 *         publisher.UpdateMetrics(metrics);
 *     }
 *
 *     publisher.Stop();
 * }
 * @endcode
 */
class DiagnosticsPublisher
{
public:
    /**
     * @brief Configuration parameters for the diagnostics publisher
     */
    struct Config
    {
        bool ros2_enabled = true;                      ///< Enable ROS2 topic publishing
        bool ros2_mirror_only = false;                 ///< Only mirror to ROS2, no internal storage
        int publish_rate_hz = 50;                      ///< Publishing frequency (Hz)
        std::string diagnostics_topic = "/go2_x5/diagnostics";  ///< ROS2 topic name
        std::string node_name = "diagnostics_publisher"; ///< ROS2 node name

        /**
         * @brief Validate configuration parameters
         * @return true if configuration is valid
         */
        bool IsValid() const
        {
            return (publish_rate_hz > 0) && (publish_rate_hz <= 1000);
        }
    };

    /**
     * @brief Constructor
     */
    DiagnosticsPublisher();

    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~DiagnosticsPublisher();

    // Non-copyable, non-movable
    DiagnosticsPublisher(const DiagnosticsPublisher&) = delete;
    DiagnosticsPublisher& operator=(const DiagnosticsPublisher&) = delete;
    DiagnosticsPublisher(DiagnosticsPublisher&&) = delete;
    DiagnosticsPublisher& operator=(DiagnosticsPublisher&&) = delete;

    /**
     * @brief Initialize the publisher with configuration
     *
     * Initializes ROS2 components if enabled and validates configuration.
     * Must be called before Start().
     *
     * @param config Configuration parameters
     * @return true if initialization successful
     */
    bool Initialize(const Config& config);

    /**
     * @brief Start the diagnostics publishing thread
     *
     * Begins periodic publishing of metrics at the configured rate.
     * Returns immediately; publishing happens in background thread.
     *
     * @return true if started successfully
     */
    bool Start();

    /**
     * @brief Stop the diagnostics publishing thread
     *
     * Signals the publishing thread to stop and waits for completion.
     * Safe to call multiple times.
     */
    void Stop();

    /**
     * @brief Update metrics from calling thread
     *
     * Thread-safe update of the current metrics snapshot.
     * Can be called from policy thread, coordinator thread, or any other thread.
     *
     * @param metrics New metrics values to store
     */
    void UpdateMetrics(const DiagnosticMetrics& metrics);

    /**
     * @brief Get a snapshot of current metrics
     *
     * Thread-safe read of the latest metrics.
     *
     * @param metrics Output parameter to receive metrics copy
     * @return true if metrics are available (publisher initialized)
     */
    bool GetMetrics(DiagnosticMetrics& metrics) const;

    /**
     * @brief Check if publisher is currently running
     * @return true if publishing thread is active
     */
    bool IsRunning() const { return running_.load(); }

    /**
     * @brief Check if publisher is initialized
     * @return true if Initialize() was successful
     */
    bool IsInitialized() const { return initialized_.load(); }

    /**
     * @brief Get the current configuration
     * @return Copy of current configuration
     */
    Config GetConfig() const;

    /**
     * @brief Get publish statistics
     */
    struct PublishStats
    {
        uint64_t total_publish_count = 0;    ///< Total number of publishes
        uint64_t failed_publish_count = 0;   ///< Number of failed publishes
        double actual_frequency_hz = 0.0;    ///< Measured publish frequency
    };
    PublishStats GetStats() const;

    /**
     * @brief Reset statistics counters
     */
    void ResetStats();

private:
    // Configuration
    Config config_;

    // State
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};

    // Thread management
    std::thread publish_thread_;
    std::mutex start_stop_mutex_;

    // Metrics storage (double buffer for thread safety)
    DiagnosticMetrics metrics_buffer_;
    mutable std::shared_mutex metrics_mutex_;

    // Publish statistics
    std::atomic<uint64_t> publish_count_{0};
    std::atomic<uint64_t> failed_publish_count_{0};
    std::chrono::steady_clock::time_point last_publish_time_;
    mutable std::mutex stats_mutex_;

    // ROS2 components (conditional)
#ifdef USE_ROS2
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
#endif  // USE_ROS2

    // Thread synchronization
    std::condition_variable_any cv_;
    std::atomic<bool> should_stop_{false};

    // Private methods

    /**
     * @brief Publish thread main function
     */
    void PublishThread();

    /**
     * @brief Publish metrics via ROS2 (if enabled)
     * @param metrics Metrics to publish
     * @return true if publish successful
     */
    bool PublishViaROS2(const DiagnosticMetrics& metrics);

    /**
     * @brief Convert metrics to JSON string for publishing
     * @param metrics Metrics to serialize
     * @return JSON string representation
     */
    static std::string MetricsToJSON(const DiagnosticMetrics& metrics);

    /**
     * @brief Get current monotonic timestamp in nanoseconds
     * @return Timestamp in nanoseconds
     */
    static uint64_t GetMonotonicTimestamp();

    /**
     * @brief Compute measured frequency from timestamps
     * @param last_time Previous publish timestamp
     * @param current_time Current publish timestamp
     * @return Computed frequency in Hz
     */
    static double ComputeFrequency(
        const std::chrono::steady_clock::time_point& last_time,
        const std::chrono::steady_clock::time_point& current_time);
};

}  // namespace rl_sar::diagnostics

#endif  // RL_SAR_DIAGNOSTICS_DIAGNOSTICS_PUBLISHER_HPP
