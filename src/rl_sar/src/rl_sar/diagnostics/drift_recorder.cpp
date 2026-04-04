#include "rl_sar/diagnostics/drift_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace rl_sar::diagnostics
{

DriftMetricsRecorder::~DriftMetricsRecorder()
{
    // Ensure clean shutdown
    recording_enabled_.store(false);
}

bool DriftMetricsRecorder::Initialize(const Config& config)
{
    config_ = config;

    // Pre-allocate snapshot buffer for circular buffer behavior
    snapshots_.reserve(config_.max_snapshots);
    snapshots_.clear();

    // Reset state
    recording_enabled_.store(false);
    zero_cmd_active_.store(false);
    snapshot_count_.store(0);
    current_seq_.store(0);
    reference_state_ = ReferenceState{};

    return true;
}

void DriftMetricsRecorder::Update(const protocol::BodyStateFrame& body_state,
                                   const protocol::ArmStateFrame& arm_state,
                                   const CommandContext& cmd_ctx)
{
    // Check if command is zero
    const bool is_zero_cmd = IsZeroCommand(cmd_ctx);
    const bool was_zero_cmd = zero_cmd_active_.load();

    // Handle state transitions for zero command detection
    if (is_zero_cmd && !was_zero_cmd) {
        // Entering zero command state - capture reference point
        UpdateReferenceState(body_state);
        zero_cmd_active_.store(true);
    } else if (!is_zero_cmd && was_zero_cmd) {
        // Leaving zero command state
        zero_cmd_active_.store(false);
    }

    // Only record if we have a valid reference and recording is enabled
    if (!recording_enabled_.load() || !reference_state_.valid) {
        return;
    }

    // Create snapshot
    DriftSnapshot snapshot = CreateSnapshot(body_state, arm_state, cmd_ctx);

    // Thread-safe insertion into circular buffer
    {
        std::lock_guard<std::mutex> lock(snapshots_mutex_);

        if (snapshots_.size() < config_.max_snapshots) {
            snapshots_.push_back(snapshot);
        } else {
            // Circular buffer: overwrite oldest
            const size_t write_pos = snapshot_count_.load() % config_.max_snapshots;
            snapshots_[write_pos] = snapshot;
        }
    }

    snapshot_count_.fetch_add(1);
    current_seq_.fetch_add(1);
}

void DriftMetricsRecorder::StartRecordingWindow()
{
    // Clear previous data
    Clear();

    // Reset reference state
    reference_state_ = ReferenceState{};

    // Enable recording
    recording_enabled_.store(true);
}

void DriftMetricsRecorder::StopRecordingWindow()
{
    recording_enabled_.store(false);
    zero_cmd_active_.store(false);
}

DriftStatistics DriftMetricsRecorder::GetStatistics() const
{
    DriftStatistics stats;

    std::lock_guard<std::mutex> lock(snapshots_mutex_);

    if (snapshots_.empty()) {
        return stats;
    }

    stats.sample_count = snapshots_.size();

    // Initialize min/max
    stats.max_xy_drift = 0.0;
    stats.max_yaw_drift = 0.0;

    // Accumulate for mean calculation
    double sum_xy = 0.0;
    double sum_yaw = 0.0;

    for (const auto& snapshot : snapshots_) {
        sum_xy += snapshot.xy_drift;
        sum_yaw += snapshot.yaw_drift;

        if (snapshot.xy_drift > stats.max_xy_drift) {
            stats.max_xy_drift = snapshot.xy_drift;
        }
        if (snapshot.yaw_drift > stats.max_yaw_drift) {
            stats.max_yaw_drift = snapshot.yaw_drift;
        }
    }

    // Compute mean
    stats.mean_xy_drift = sum_xy / static_cast<double>(stats.sample_count);
    stats.mean_yaw_drift = sum_yaw / static_cast<double>(stats.sample_count);

    // Compute standard deviation
    double sum_sq_diff_xy = 0.0;
    double sum_sq_diff_yaw = 0.0;

    for (const auto& snapshot : snapshots_) {
        const double diff_xy = snapshot.xy_drift - stats.mean_xy_drift;
        const double diff_yaw = snapshot.yaw_drift - stats.mean_yaw_drift;
        sum_sq_diff_xy += diff_xy * diff_xy;
        sum_sq_diff_yaw += diff_yaw * diff_yaw;
    }

    stats.std_xy_drift = std::sqrt(sum_sq_diff_xy / static_cast<double>(stats.sample_count));
    stats.std_yaw_drift = std::sqrt(sum_sq_diff_yaw / static_cast<double>(stats.sample_count));

    return stats;
}

std::vector<DriftSnapshot> DriftMetricsRecorder::GetSnapshots() const
{
    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    return snapshots_;  // Return by value for thread safety
}

bool DriftMetricsRecorder::ExportToCSV(const std::string& path) const
{
    const std::string output_path = path.empty() ? config_.output_csv_path : path;

    std::lock_guard<std::mutex> lock(snapshots_mutex_);

    std::ofstream out_file(output_path);
    if (!out_file.is_open()) {
        return false;
    }

    // Write header
    out_file << "timestamp_ns,seq,base_x,base_y,base_yaw,ref_x,ref_y,ref_yaw,"
             << "xy_drift,yaw_drift,"
             << "arm_j0,arm_j1,arm_j2,arm_j3,arm_j4,arm_j5,"
             << "arm_cmd0,arm_cmd1,arm_cmd2,arm_cmd3,arm_cmd4,arm_cmd5,"
             << "arm_tracking_error,zero_lin_vel,zero_ang_vel,cmd_x,cmd_y,cmd_yaw\n";

    // Write data rows
    for (const auto& snapshot : snapshots_) {
        out_file << snapshot.timestamp_ns << ","
                 << snapshot.seq << ","
                 << std::fixed << std::setprecision(6)
                 << snapshot.base_x << ","
                 << snapshot.base_y << ","
                 << snapshot.base_yaw << ","
                 << snapshot.ref_x << ","
                 << snapshot.ref_y << ","
                 << snapshot.ref_yaw << ","
                 << snapshot.xy_drift << ","
                 << snapshot.yaw_drift << ",";

        // Arm joint positions
        for (size_t i = 0; i < 6; ++i) {
            out_file << snapshot.arm_joint_pos[i];
            if (i < 5) out_file << ",";
        }
        out_file << ",";

        // Arm joint commands
        for (size_t i = 0; i < 6; ++i) {
            out_file << snapshot.arm_joint_cmd[i];
            if (i < 5) out_file << ",";
        }
        out_file << ",";

        // Arm tracking error and command context
        out_file << snapshot.arm_tracking_error << ","
                 << (snapshot.zero_lin_vel_cmd ? "1" : "0") << ","
                 << (snapshot.zero_ang_vel_cmd ? "1" : "0") << ","
                 << snapshot.cmd_x << ","
                 << snapshot.cmd_y << ","
                 << snapshot.cmd_yaw << "\n";
    }

    out_file.close();
    return !out_file.fail();
}

void DriftMetricsRecorder::Clear()
{
    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    snapshots_.clear();
    snapshot_count_.store(0);
    current_seq_.store(0);
    reference_state_ = ReferenceState{};
}

DriftMetricsRecorder::Config DriftMetricsRecorder::GetConfig() const
{
    return config_;
}

bool DriftMetricsRecorder::IsZeroCommand(const CommandContext& cmd_ctx) const
{
    const double lin_vel_mag = std::sqrt(
        cmd_ctx.cmd_x * cmd_ctx.cmd_x + cmd_ctx.cmd_y * cmd_ctx.cmd_y);
    const double ang_vel_mag = std::abs(cmd_ctx.cmd_yaw);

    return (lin_vel_mag < config_.zero_cmd_threshold) &&
           (ang_vel_mag < config_.zero_cmd_threshold);
}

void DriftMetricsRecorder::UpdateReferenceState(const protocol::BodyStateFrame& body_state)
{
    // Note: BodyStateFrame contains velocity and IMU data but not absolute position
    // In a real implementation, position would come from state estimation or odometry
    // For now, we initialize with zero and expect integration in the calling context
    reference_state_.x = 0.0;
    reference_state_.y = 0.0;
    reference_state_.yaw = 0.0;
    reference_state_.valid = true;
}

DriftSnapshot DriftMetricsRecorder::CreateSnapshot(
    const protocol::BodyStateFrame& body_state,
    const protocol::ArmStateFrame& arm_state,
    const CommandContext& cmd_ctx) const
{
    DriftSnapshot snapshot;

    snapshot.timestamp_ns = body_state.header.source_monotonic_ns;
    snapshot.seq = current_seq_.load();

    // Base state - in real implementation, integrate from odometry
    // For now, using velocity as proxy (caller should provide actual position)
    snapshot.base_x = 0.0;
    snapshot.base_y = 0.0;
    snapshot.base_yaw = 0.0;

    // Reference state
    snapshot.ref_x = reference_state_.x;
    snapshot.ref_y = reference_state_.y;
    snapshot.ref_yaw = reference_state_.yaw;

    // Computed drift
    snapshot.xy_drift = ComputeXYDrift(snapshot.base_x, snapshot.base_y,
                                       snapshot.ref_x, snapshot.ref_y);
    snapshot.yaw_drift = ComputeYawDrift(snapshot.base_yaw, snapshot.ref_yaw);

    // Arm state
    for (size_t i = 0; i < 6; ++i) {
        snapshot.arm_joint_pos[i] = static_cast<double>(arm_state.q[i]);
        snapshot.arm_joint_cmd[i] = static_cast<double>(arm_state.q_target[i]);
    }
    snapshot.arm_tracking_error = ComputeArmTrackingError(arm_state);

    // Command context
    const bool is_zero = IsZeroCommand(cmd_ctx);
    snapshot.zero_lin_vel_cmd = is_zero;
    snapshot.zero_ang_vel_cmd = is_zero;
    snapshot.cmd_x = cmd_ctx.cmd_x;
    snapshot.cmd_y = cmd_ctx.cmd_y;
    snapshot.cmd_yaw = cmd_ctx.cmd_yaw;

    return snapshot;
}

double DriftMetricsRecorder::ComputeXYDrift(double x, double y, double ref_x, double ref_y) const
{
    const double dx = x - ref_x;
    const double dy = y - ref_y;
    return std::sqrt(dx * dx + dy * dy);
}

double DriftMetricsRecorder::ComputeYawDrift(double yaw, double ref_yaw) const
{
    double normalized_yaw = yaw;
    double normalized_ref = ref_yaw;
    NormalizeYaw(normalized_yaw);
    NormalizeYaw(normalized_ref);

    double diff = normalized_yaw - normalized_ref;
    NormalizeYaw(diff);

    return std::abs(diff);
}

double DriftMetricsRecorder::ComputeArmTrackingError(const protocol::ArmStateFrame& arm_state) const
{
    // Compute L2 norm of tracking error
    double sum_sq = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        const double error = static_cast<double>(arm_state.tracking_error[i]);
        sum_sq += error * error;
    }
    return std::sqrt(sum_sq);
}

void DriftMetricsRecorder::NormalizeYaw(double& yaw) const
{
    // Normalize to [-pi, pi]
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
}

}  // namespace rl_sar::diagnostics
