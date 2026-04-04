#ifndef RL_SAR_DIAGNOSTICS_DRIFT_RECORDER_HPP
#define RL_SAR_DIAGNOSTICS_DRIFT_RECORDER_HPP

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

namespace rl_sar::diagnostics
{

/**
 * @brief Snapshot of drift metrics at a single point in time
 *
 * Captures the complete state for drift analysis including base position,
 * reference points, computed drift values, and arm context.
 */
struct DriftSnapshot
{
    uint64_t timestamp_ns;       ///< Monotonic timestamp of the snapshot
    uint64_t seq;                ///< Sequence number for continuity tracking

    // Base state
    double base_x;               ///< Current base X position (meters)
    double base_y;               ///< Current base Y position (meters)
    double base_yaw;             ///< Current base yaw angle (radians)

    // Reference point (when zero command started)
    double ref_x;                ///< Reference X position (meters)
    double ref_y;                ///< Reference Y position (meters)
    double ref_yaw;              ///< Reference yaw angle (radians)

    // Computed drift
    double xy_drift;             ///< Euclidean distance from reference (meters)
    double yaw_drift;            ///< Absolute angular difference from reference (radians)

    // Arm context
    std::array<double, 6> arm_joint_pos;    ///< Current arm joint positions (rad)
    std::array<double, 6> arm_joint_cmd;    ///< Current arm joint commands (rad)
    double arm_tracking_error;              ///< L2 norm of arm tracking error

    // Command context
    bool zero_lin_vel_cmd;       ///< True if linear velocity command is near zero
    bool zero_ang_vel_cmd;       ///< True if angular velocity command is near zero
    double cmd_x;                ///< Current linear velocity command X (m/s)
    double cmd_y;                ///< Current linear velocity command Y (m/s)
    double cmd_yaw;              ///< Current angular velocity command (rad/s)
};

/**
 * @brief Statistical summary of drift metrics over a recording window
 */
struct DriftStatistics
{
    size_t sample_count;         ///< Total number of snapshots in the window
    double max_xy_drift;         ///< Maximum XY drift observed (meters)
    double max_yaw_drift;        ///< Maximum yaw drift observed (radians)
    double mean_xy_drift;        ///< Mean XY drift (meters)
    double mean_yaw_drift;       ///< Mean yaw drift (radians)
    double std_xy_drift;         ///< Standard deviation of XY drift (meters)
    double std_yaw_drift;        ///< Standard deviation of yaw drift (radians)

    /**
     * @brief Default constructor initializing all fields to zero
     */
    DriftStatistics()
        : sample_count(0)
        , max_xy_drift(0.0)
        , max_yaw_drift(0.0)
        , mean_xy_drift(0.0)
        , mean_yaw_drift(0.0)
        , std_xy_drift(0.0)
        , std_yaw_drift(0.0)
    {}
};

/**
 * @brief Command context wrapper for drift recorder updates
 *
 * Provides a unified interface for command information from various sources
 * (operator commands, policy commands, etc.)
 */
struct CommandContext
{
    double cmd_x;                ///< Linear velocity command X (m/s)
    double cmd_y;                ///< Linear velocity command Y (m/s)
    double cmd_yaw;              ///< Angular velocity command (rad/s)
    uint64_t timestamp_ns;       ///< Command timestamp

    CommandContext(double x = 0.0, double y = 0.0, double yaw = 0.0, uint64_t ts = 0)
        : cmd_x(x), cmd_y(y), cmd_yaw(yaw), timestamp_ns(ts)
    {}
};

/**
 * @brief Drift metrics recorder for sim2real validation
 *
 * Records xy and yaw drift during zero command conditions, correlating
 * drift with arm motion context. This component is essential for validating
 * simulation-to-real transfer fidelity.
 *
 * Key features:
 * - Automatic detection of zero command conditions
 * - Reference point capture when entering zero command state
 * - Thread-safe snapshot buffer with configurable maximum size
 * - Windowed statistics for controlled measurement intervals
 * - CSV export for offline analysis
 *
 * Usage example:
 * @code
 * DriftMetricsRecorder recorder;
 * DriftMetricsRecorder::Config config;
 * config.zero_cmd_threshold = 0.01;
 * config.max_snapshots = 10000;
 * recorder.Initialize(config);
 *
 * recorder.StartRecordingWindow();
 *
 * while (recording) {
 *     recorder.Update(body_state, arm_state, cmd_ctx);
 * }
 *
 * recorder.StopRecordingWindow();
 * auto stats = recorder.GetStatistics();
 * recorder.ExportToCSV("/tmp/drift_data.csv");
 * @endcode
 */
class DriftMetricsRecorder
{
public:
    /**
     * @brief Configuration parameters for the drift recorder
     */
    struct Config
    {
        double zero_cmd_threshold = 0.01;      ///< Command magnitude below this is considered zero (m/s, rad/s)
        size_t max_snapshots = 10000;          ///< Maximum number of snapshots to retain in circular buffer
        std::string output_csv_path = "/tmp/go2_x5_drift.csv";  ///< Default path for CSV export
        bool auto_start_on_zero_cmd = false;   ///< Automatically start recording when zero command detected
    };

    /**
     * @brief Constructor
     */
    DriftMetricsRecorder() = default;

    /**
     * @brief Destructor
     */
    ~DriftMetricsRecorder();

    /**
     * @brief Initialize the recorder with configuration
     *
     * @param config Configuration parameters
     * @return true if initialization successful
     */
    bool Initialize(const Config& config);

    /**
     * @brief Update recorder with current state
     *
     * Records a snapshot if recording is active. Automatically manages
     * reference point capture when transitioning into zero command state.
     *
     * @param body_state Current body state from protocol
     * @param arm_state Current arm state from protocol
     * @param cmd_ctx Current command context
     */
    void Update(const protocol::BodyStateFrame& body_state,
                const protocol::ArmStateFrame& arm_state,
                const CommandContext& cmd_ctx);

    /**
     * @brief Start a new recording window
     *
     * Clears previous reference state and begins accumulating snapshots.
     * Call this before a test sequence to establish a clean baseline.
     */
    void StartRecordingWindow();

    /**
     * @brief Stop the current recording window
     *
     * Stops snapshot accumulation but retains recorded data for analysis.
     */
    void StopRecordingWindow();

    /**
     * @brief Check if recording is currently active
     *
     * @return true if recording window is open
     */
    bool IsRecording() const { return recording_enabled_.load(); }

    /**
     * @brief Get statistics for the current recording window
     *
     * Computes statistical summary of all recorded drift snapshots.
     *
     * @return DriftStatistics structure with computed metrics
     */
    DriftStatistics GetStatistics() const;

    /**
     * @brief Get all recorded snapshots
     *
     * Returns a copy of the current snapshot buffer.
     *
     * @return Vector of DriftSnapshot copies
     */
    std::vector<DriftSnapshot> GetSnapshots() const;

    /**
     * @brief Export recorded data to CSV file
     *
     * Writes all snapshots to a CSV file with header row for analysis.
     *
     * @param path Output file path, or uses configured default if empty
     * @return true if export successful
     */
    bool ExportToCSV(const std::string& path = "") const;

    /**
     * @brief Clear all recorded data and reset state
     *
     * Resets all buffers, reference points, and statistics.
     */
    void Clear();

    /**
     * @brief Get the current configuration
     *
     * @return Copy of current configuration
     */
    Config GetConfig() const;

    /**
     * @brief Get the number of snapshots currently recorded
     *
     * @return Current snapshot count
     */
    size_t GetSnapshotCount() const { return snapshot_count_.load(); }

private:
    // Configuration
    Config config_;

    // Recording state
    std::atomic<bool> recording_enabled_{false};
    std::atomic<bool> zero_cmd_active_{false};
    std::atomic<size_t> snapshot_count_{0};
    std::atomic<uint64_t> current_seq_{0};

    // Reference point (captured when zero command starts)
    struct ReferenceState
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        bool valid = false;
    };
    ReferenceState reference_state_;

    // Snapshot buffer (thread-safe circular buffer)
    mutable std::mutex snapshots_mutex_;
    std::vector<DriftSnapshot> snapshots_;

    // Helper methods
    bool IsZeroCommand(const CommandContext& cmd_ctx) const;
    void UpdateReferenceState(const protocol::BodyStateFrame& body_state);
    DriftSnapshot CreateSnapshot(const protocol::BodyStateFrame& body_state,
                                 const protocol::ArmStateFrame& arm_state,
                                 const CommandContext& cmd_ctx) const;
    double ComputeXYDrift(double x, double y, double ref_x, double ref_y) const;
    double ComputeYawDrift(double yaw, double ref_yaw) const;
    double ComputeArmTrackingError(const protocol::ArmStateFrame& arm_state) const;
    void NormalizeYaw(double& yaw) const;
};

}  // namespace rl_sar::diagnostics

#endif  // RL_SAR_DIAGNOSTICS_DRIFT_RECORDER_HPP
