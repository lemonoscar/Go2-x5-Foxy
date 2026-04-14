#include "rl_sar/diagnostics/drift_data_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace rl_sar::diagnostics
{

namespace
{

float VectorNorm3(const std::array<float, 3>& value)
{
    return std::sqrt(
        value[0] * value[0] +
        value[1] * value[1] +
        value[2] * value[2]);
}

}  // namespace

DriftDataRecorder::DriftDataRecorder(size_t max_snapshots)
    : max_snapshots_(std::max<size_t>(1, max_snapshots))
{
}

void DriftDataRecorder::SetMaxSnapshots(const size_t max_snapshots)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->max_snapshots_ = std::max<size_t>(1, max_snapshots);
    while (this->snapshots_.size() > this->max_snapshots_)
    {
        this->snapshots_.pop_front();
    }
}

void DriftDataRecorder::RecordSnapshot(const Snapshot& snapshot)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->snapshots_.size() >= this->max_snapshots_)
    {
        this->snapshots_.pop_front();
    }
    this->snapshots_.push_back(snapshot);
}

bool DriftDataRecorder::SaveToFile(const std::string& path) const
{
    if (path.empty())
    {
        return false;
    }

    std::vector<Snapshot> snapshots = this->GetSnapshots();
    std::ofstream out(path);
    if (!out.is_open())
    {
        return false;
    }

    out << "timestamp_ns,zero_velocity,support_feet_mask,"
        << "cmd_x,cmd_y,cmd_yaw,"
        << "est_vx,est_vy,est_vz,"
        << "imu_wx,imu_wy,imu_wz,"
        << "imu_ax,imu_ay,imu_az";
    for (size_t i = 0; i < 12; ++i)
    {
        out << ",joint_q_" << i;
    }
    out << '\n';

    out << std::fixed << std::setprecision(6);
    for (const auto& snapshot : snapshots)
    {
        out << snapshot.timestamp_ns << ','
            << (snapshot.is_zero_velocity_command ? 1 : 0) << ','
            << snapshot.support_feet_mask << ','
            << snapshot.velocity_command[0] << ','
            << snapshot.velocity_command[1] << ','
            << snapshot.velocity_command[2] << ','
            << snapshot.estimated_base_velocity[0] << ','
            << snapshot.estimated_base_velocity[1] << ','
            << snapshot.estimated_base_velocity[2] << ','
            << snapshot.imu_angular_velocity[0] << ','
            << snapshot.imu_angular_velocity[1] << ','
            << snapshot.imu_angular_velocity[2] << ','
            << snapshot.imu_acceleration[0] << ','
            << snapshot.imu_acceleration[1] << ','
            << snapshot.imu_acceleration[2];
        for (float joint_position : snapshot.joint_positions)
        {
            out << ',' << joint_position;
        }
        out << '\n';
    }

    return out.good();
}

std::string DriftDataRecorder::GetWindowSummary() const
{
    std::vector<Snapshot> snapshots = this->GetSnapshots();
    std::ostringstream oss;
    if (snapshots.empty())
    {
        oss << "samples=0";
        return oss.str();
    }

    size_t zero_cmd_count = 0;
    float max_base_speed = 0.0f;
    float mean_base_speed = 0.0f;
    for (const auto& snapshot : snapshots)
    {
        if (snapshot.is_zero_velocity_command)
        {
            ++zero_cmd_count;
        }
        const float speed = VectorNorm3(snapshot.estimated_base_velocity);
        max_base_speed = std::max(max_base_speed, speed);
        mean_base_speed += speed;
    }
    mean_base_speed /= static_cast<float>(snapshots.size());

    oss << std::fixed << std::setprecision(4)
        << "samples=" << snapshots.size()
        << ", zero_cmd=" << zero_cmd_count
        << ", mean_est_speed=" << mean_base_speed
        << ", max_est_speed=" << max_base_speed
        << ", latest_support_mask=" << snapshots.back().support_feet_mask;
    return oss.str();
}

std::vector<DriftDataRecorder::Snapshot> DriftDataRecorder::GetSnapshots() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return std::vector<Snapshot>(this->snapshots_.begin(), this->snapshots_.end());
}

size_t DriftDataRecorder::GetSnapshotCount() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->snapshots_.size();
}

void DriftDataRecorder::Clear()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->snapshots_.clear();
}

}  // namespace rl_sar::diagnostics
