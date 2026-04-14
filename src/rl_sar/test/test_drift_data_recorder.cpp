#include <cstdio>
#include <iostream>
#include <string>

#include "rl_sar/diagnostics/drift_data_recorder.hpp"

namespace
{

using rl_sar::diagnostics::DriftDataRecorder;

bool ExpectTrue(bool condition, const char* name)
{
    if (!condition)
    {
        std::cerr << name << " was not true\n";
        return false;
    }
    return true;
}

bool ExpectEq(size_t lhs, size_t rhs, const char* name)
{
    if (lhs != rhs)
    {
        std::cerr << name << " mismatch: " << lhs << " vs " << rhs << '\n';
        return false;
    }
    return true;
}

bool TestRetentionLimit()
{
    std::cout << "Testing DriftDataRecorder retention... ";
    DriftDataRecorder recorder(3);
    for (uint64_t i = 0; i < 5; ++i)
    {
        DriftDataRecorder::Snapshot snapshot;
        snapshot.timestamp_ns = i;
        snapshot.is_zero_velocity_command = true;
        recorder.RecordSnapshot(snapshot);
    }

    if (!ExpectEq(recorder.GetSnapshotCount(), 3, "snapshot_count")) { return false; }
    const auto snapshots = recorder.GetSnapshots();
    if (!ExpectEq(snapshots.front().timestamp_ns, 2u, "oldest_timestamp")) { return false; }
    if (!ExpectEq(snapshots.back().timestamp_ns, 4u, "latest_timestamp")) { return false; }
    std::cout << "PASS\n";
    return true;
}

bool TestSummaryAndSave()
{
    std::cout << "Testing DriftDataRecorder summary/save... ";
    DriftDataRecorder recorder;
    DriftDataRecorder::Snapshot snapshot;
    snapshot.timestamp_ns = 123;
    snapshot.is_zero_velocity_command = true;
    snapshot.estimated_base_velocity = {0.1f, 0.2f, 0.0f};
    snapshot.support_feet_mask = 0x3;
    recorder.RecordSnapshot(snapshot);

    const std::string summary = recorder.GetWindowSummary();
    if (!ExpectTrue(summary.find("samples=1") != std::string::npos, "summary sample count")) { return false; }
    if (!ExpectTrue(summary.find("latest_support_mask=3") != std::string::npos, "summary support mask")) { return false; }

    const std::string path = "/tmp/test_drift_data_recorder.csv";
    std::remove(path.c_str());
    if (!ExpectTrue(recorder.SaveToFile(path), "save_to_file")) { return false; }
    std::cout << "PASS\n";
    return true;
}

}  // namespace

int main()
{
    if (!TestRetentionLimit()) { return 1; }
    if (!TestSummaryAndSave()) { return 1; }
    std::cout << "All DriftDataRecorder tests passed.\n";
    return 0;
}
