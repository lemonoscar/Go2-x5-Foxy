#ifndef RL_SAR_DIAGNOSTICS_DRIFT_DATA_RECORDER_HPP
#define RL_SAR_DIAGNOSTICS_DRIFT_DATA_RECORDER_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace rl_sar::diagnostics
{

class DriftDataRecorder
{
public:
    struct Snapshot
    {
        uint64_t timestamp_ns = 0;
        std::array<float, 12> joint_positions{};
        std::array<float, 3> imu_angular_velocity{};
        std::array<float, 3> imu_acceleration{};
        std::array<float, 3> velocity_command{};
        std::array<float, 3> estimated_base_velocity{};
        uint32_t support_feet_mask = 0;
        bool is_zero_velocity_command = false;
    };

    explicit DriftDataRecorder(size_t max_snapshots = 10000);

    void SetMaxSnapshots(size_t max_snapshots);
    void RecordSnapshot(const Snapshot& snapshot);
    bool SaveToFile(const std::string& path) const;
    std::string GetWindowSummary() const;
    std::vector<Snapshot> GetSnapshots() const;
    size_t GetSnapshotCount() const;
    void Clear();

private:
    mutable std::mutex mutex_;
    std::deque<Snapshot> snapshots_;
    size_t max_snapshots_ = 10000;
};

}  // namespace rl_sar::diagnostics

#endif  // RL_SAR_DIAGNOSTICS_DRIFT_DATA_RECORDER_HPP
