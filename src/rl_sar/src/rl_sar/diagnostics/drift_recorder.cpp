#include "rl_sar/diagnostics/drift_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace rl_sar::diagnostics
{

namespace
{

constexpr double kNsToSeconds = 1e-9;
constexpr double kQuaternionNormEpsilon = 1e-6;
constexpr double kMaxIntegrationDtSec = 0.25;

bool IsFinite(const double value)
{
    return std::isfinite(value);
}

bool IsQuaternionUsable(const std::array<float, 4>& quat)
{
    double norm_sq = 0.0;
    for (float value : quat)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
        norm_sq += static_cast<double>(value) * static_cast<double>(value);
    }
    return norm_sq > kQuaternionNormEpsilon;
}

std::array<double, 4> NormalizeQuaternion(const std::array<float, 4>& quat)
{
    double norm_sq = 0.0;
    for (float value : quat)
    {
        norm_sq += static_cast<double>(value) * static_cast<double>(value);
    }

    if (norm_sq <= kQuaternionNormEpsilon)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }

    const double inv_norm = 1.0 / std::sqrt(norm_sq);
    return {
        static_cast<double>(quat[0]) * inv_norm,
        static_cast<double>(quat[1]) * inv_norm,
        static_cast<double>(quat[2]) * inv_norm,
        static_cast<double>(quat[3]) * inv_norm,
    };
}

double QuaternionToYaw(const std::array<float, 4>& quat)
{
    const auto q = NormalizeQuaternion(quat);
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 3> RotateBodyToWorld(const std::array<float, 4>& quat,
                                        const std::array<float, 3>& vec)
{
    const auto q = NormalizeQuaternion(quat);
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    const double vx = static_cast<double>(vec[0]);
    const double vy = static_cast<double>(vec[1]);
    const double vz = static_cast<double>(vec[2]);

    const double a_x = vx * (2.0 * w * w - 1.0);
    const double a_y = vy * (2.0 * w * w - 1.0);
    const double a_z = vz * (2.0 * w * w - 1.0);

    const double cross_x = y * vz - z * vy;
    const double cross_y = z * vx - x * vz;
    const double cross_z = x * vy - y * vx;

    const double b_x = cross_x * w * 2.0;
    const double b_y = cross_y * w * 2.0;
    const double b_z = cross_z * w * 2.0;

    const double dot = x * vx + y * vy + z * vz;

    const double c_x = x * dot * 2.0;
    const double c_y = y * dot * 2.0;
    const double c_z = z * dot * 2.0;

    return {a_x + b_x + c_x, a_y + b_y + c_y, a_z + b_z + c_z};
}

bool HasUsablePlanarVelocity(const protocol::BodyStateFrame& body_state)
{
    if ((body_state.header.validity_flags & protocol::kValidityPayloadValid) == 0U)
    {
        return false;
    }
    if ((body_state.header.validity_flags & protocol::kValidityPartial) != 0U)
    {
        return false;
    }

    for (float value : body_state.base_lin_vel)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

}  // namespace

DriftMetricsRecorder::~DriftMetricsRecorder()
{
    recording_enabled_.store(false);
}

bool DriftMetricsRecorder::Initialize(const Config& config)
{
    config_ = config;
    if (config_.max_snapshots == 0)
    {
        config_.max_snapshots = 1;
    }
    snapshots_.reserve(config_.max_snapshots);
    snapshots_.clear();
    recording_enabled_.store(false);
    zero_cmd_active_.store(false);
    snapshot_count_.store(0);
    current_seq_.store(0);
    reference_state_ = ReferenceState{};
    pose_state_ = PoseEstimateState{};
    return true;
}

void DriftMetricsRecorder::Update(const protocol::BodyStateFrame& body_state,
                                  const protocol::ArmStateFrame& arm_state,
                                  const CommandContext& cmd_ctx)
{
    UpdatePoseEstimate(body_state);

    const bool is_zero_cmd = IsZeroCommand(cmd_ctx);
    const bool was_zero_cmd = zero_cmd_active_.load();
    if (is_zero_cmd && !was_zero_cmd)
    {
        UpdateReferenceState();
        zero_cmd_active_.store(true);
    }
    else if (!is_zero_cmd && was_zero_cmd)
    {
        zero_cmd_active_.store(false);
    }

    if (!recording_enabled_.load() || !reference_state_.valid)
    {
        return;
    }

    const DriftSnapshot snapshot = CreateSnapshot(body_state, arm_state, cmd_ctx);
    {
        std::lock_guard<std::mutex> lock(snapshots_mutex_);
        if (snapshots_.size() < config_.max_snapshots)
        {
            snapshots_.push_back(snapshot);
        }
        else
        {
            const size_t write_pos = snapshot_count_.load() % config_.max_snapshots;
            snapshots_[write_pos] = snapshot;
        }
    }

    snapshot_count_.fetch_add(1);
    current_seq_.fetch_add(1);
}

void DriftMetricsRecorder::StartRecordingWindow()
{
    Clear();
    reference_state_ = ReferenceState{};
    pose_state_ = PoseEstimateState{};
    zero_cmd_active_.store(false);
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
    if (snapshots_.empty())
    {
        return stats;
    }

    stats.sample_count = snapshots_.size();
    double sum_xy = 0.0;
    double sum_yaw = 0.0;
    size_t xy_count = 0;
    size_t yaw_count = 0;

    for (const auto& snapshot : snapshots_)
    {
        if (snapshot.xy_valid)
        {
            sum_xy += snapshot.xy_drift;
            ++xy_count;
            if (snapshot.xy_drift > stats.max_xy_drift)
            {
                stats.max_xy_drift = snapshot.xy_drift;
            }
        }
        if (snapshot.yaw_valid)
        {
            sum_yaw += snapshot.yaw_drift;
            ++yaw_count;
            if (snapshot.yaw_drift > stats.max_yaw_drift)
            {
                stats.max_yaw_drift = snapshot.yaw_drift;
            }
        }
    }

    stats.mean_xy_drift = xy_count > 0 ? (sum_xy / static_cast<double>(xy_count)) : 0.0;
    stats.mean_yaw_drift = yaw_count > 0 ? (sum_yaw / static_cast<double>(yaw_count)) : 0.0;

    double sum_sq_diff_xy = 0.0;
    double sum_sq_diff_yaw = 0.0;
    for (const auto& snapshot : snapshots_)
    {
        if (snapshot.xy_valid)
        {
            const double diff_xy = snapshot.xy_drift - stats.mean_xy_drift;
            sum_sq_diff_xy += diff_xy * diff_xy;
        }
        if (snapshot.yaw_valid)
        {
            const double diff_yaw = snapshot.yaw_drift - stats.mean_yaw_drift;
            sum_sq_diff_yaw += diff_yaw * diff_yaw;
        }
    }

    stats.std_xy_drift =
        xy_count > 0 ? std::sqrt(sum_sq_diff_xy / static_cast<double>(xy_count)) : 0.0;
    stats.std_yaw_drift =
        yaw_count > 0 ? std::sqrt(sum_sq_diff_yaw / static_cast<double>(yaw_count)) : 0.0;
    return stats;
}

std::vector<DriftSnapshot> DriftMetricsRecorder::GetSnapshots() const
{
    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    if (snapshots_.empty() || snapshot_count_.load() <= snapshots_.size())
    {
        return snapshots_;
    }

    std::vector<DriftSnapshot> ordered;
    ordered.reserve(snapshots_.size());
    const size_t oldest_index = snapshot_count_.load() % config_.max_snapshots;
    for (size_t i = oldest_index; i < snapshots_.size(); ++i)
    {
        ordered.push_back(snapshots_[i]);
    }
    for (size_t i = 0; i < oldest_index; ++i)
    {
        ordered.push_back(snapshots_[i]);
    }
    return ordered;
}

bool DriftMetricsRecorder::ExportToCSV(const std::string& path) const
{
    const std::string output_path = path.empty() ? config_.output_csv_path : path;

    std::ofstream out_file(output_path);
    if (!out_file.is_open())
    {
        return false;
    }

    out_file << "timestamp_ns,seq,base_x,base_y,base_yaw,ref_x,ref_y,ref_yaw,"
             << "xy_drift,yaw_drift,xy_valid,yaw_valid,"
             << "arm_j0,arm_j1,arm_j2,arm_j3,arm_j4,arm_j5,"
             << "arm_cmd0,arm_cmd1,arm_cmd2,arm_cmd3,arm_cmd4,arm_cmd5,"
             << "arm_tracking_error,zero_lin_vel,zero_ang_vel,cmd_x,cmd_y,cmd_yaw\n";

    for (const auto& snapshot : GetSnapshots())
    {
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
                 << snapshot.yaw_drift << ","
                 << (snapshot.xy_valid ? "1" : "0") << ","
                 << (snapshot.yaw_valid ? "1" : "0") << ",";

        for (size_t i = 0; i < 6; ++i)
        {
            out_file << snapshot.arm_joint_pos[i];
            if (i < 5)
            {
                out_file << ",";
            }
        }
        out_file << ",";

        for (size_t i = 0; i < 6; ++i)
        {
            out_file << snapshot.arm_joint_cmd[i];
            if (i < 5)
            {
                out_file << ",";
            }
        }
        out_file << ",";

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
    pose_state_ = PoseEstimateState{};
    zero_cmd_active_.store(false);
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

void DriftMetricsRecorder::UpdatePoseEstimate(const protocol::BodyStateFrame& body_state)
{
    const uint64_t timestamp_ns = body_state.header.source_monotonic_ns;
    const bool yaw_valid = IsQuaternionUsable(body_state.imu_quat);
    const bool planar_velocity_valid = HasUsablePlanarVelocity(body_state) && yaw_valid;

    if (!pose_state_.initialized)
    {
        pose_state_.timestamp_ns = timestamp_ns;
        pose_state_.initialized = true;
    }

    if (yaw_valid)
    {
        pose_state_.yaw = QuaternionToYaw(body_state.imu_quat);
        pose_state_.yaw_valid = true;
    }
    else
    {
        pose_state_.yaw_valid = false;
    }

    if (planar_velocity_valid &&
        pose_state_.timestamp_ns != 0 &&
        timestamp_ns > pose_state_.timestamp_ns)
    {
        const double dt =
            static_cast<double>(timestamp_ns - pose_state_.timestamp_ns) * kNsToSeconds;
        if (dt > 0.0 && dt <= kMaxIntegrationDtSec)
        {
            const auto world_vel = RotateBodyToWorld(body_state.imu_quat, body_state.base_lin_vel);
            if (IsFinite(world_vel[0]) && IsFinite(world_vel[1]))
            {
                pose_state_.x += world_vel[0] * dt;
                pose_state_.y += world_vel[1] * dt;
                pose_state_.xy_valid = true;
            }
            else
            {
                pose_state_.xy_valid = false;
            }
        }
        else
        {
            pose_state_.xy_valid = false;
        }
    }
    else if (!planar_velocity_valid)
    {
        pose_state_.xy_valid = false;
    }

    pose_state_.timestamp_ns = timestamp_ns;
}

void DriftMetricsRecorder::UpdateReferenceState()
{
    reference_state_.x = pose_state_.x;
    reference_state_.y = pose_state_.y;
    reference_state_.yaw = pose_state_.yaw;
    reference_state_.xy_valid = pose_state_.xy_valid;
    reference_state_.yaw_valid = pose_state_.yaw_valid;
    reference_state_.valid = reference_state_.xy_valid || reference_state_.yaw_valid;
}

DriftSnapshot DriftMetricsRecorder::CreateSnapshot(
    const protocol::BodyStateFrame& body_state,
    const protocol::ArmStateFrame& arm_state,
    const CommandContext& cmd_ctx) const
{
    DriftSnapshot snapshot;
    snapshot.timestamp_ns = body_state.header.source_monotonic_ns;
    snapshot.seq = current_seq_.load();

    snapshot.base_x = pose_state_.x;
    snapshot.base_y = pose_state_.y;
    snapshot.base_yaw = pose_state_.yaw;
    snapshot.ref_x = reference_state_.x;
    snapshot.ref_y = reference_state_.y;
    snapshot.ref_yaw = reference_state_.yaw;

    snapshot.xy_valid = pose_state_.xy_valid && reference_state_.xy_valid;
    snapshot.yaw_valid = pose_state_.yaw_valid && reference_state_.yaw_valid;
    snapshot.xy_drift = snapshot.xy_valid
        ? ComputeXYDrift(snapshot.base_x, snapshot.base_y, snapshot.ref_x, snapshot.ref_y)
        : 0.0;
    snapshot.yaw_drift = snapshot.yaw_valid
        ? ComputeYawDrift(snapshot.base_yaw, snapshot.ref_yaw)
        : 0.0;

    for (size_t i = 0; i < 6; ++i)
    {
        snapshot.arm_joint_pos[i] = static_cast<double>(arm_state.q[i]);
        snapshot.arm_joint_cmd[i] = static_cast<double>(arm_state.q_target[i]);
    }
    snapshot.arm_tracking_error = ComputeArmTrackingError(arm_state);

    const double lin_vel_mag = std::sqrt(
        cmd_ctx.cmd_x * cmd_ctx.cmd_x + cmd_ctx.cmd_y * cmd_ctx.cmd_y);
    snapshot.zero_lin_vel_cmd = lin_vel_mag < config_.zero_cmd_threshold;
    snapshot.zero_ang_vel_cmd = std::abs(cmd_ctx.cmd_yaw) < config_.zero_cmd_threshold;
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
    double sum_sq = 0.0;
    for (size_t i = 0; i < 6; ++i)
    {
        const double error = static_cast<double>(arm_state.tracking_error[i]);
        sum_sq += error * error;
    }
    return std::sqrt(sum_sq);
}

void DriftMetricsRecorder::NormalizeYaw(double& yaw) const
{
    while (yaw > M_PI)
    {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI)
    {
        yaw += 2.0 * M_PI;
    }
}

}  // namespace rl_sar::diagnostics
