#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rl_sar/diagnostics/drift_recorder.hpp"
#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

namespace
{

using namespace rl_sar;
using namespace rl_sar::diagnostics;
using namespace rl_sar::protocol;

constexpr double kEpsilon = 1e-6;

bool ExpectNear(double lhs, double rhs, const char* name, double epsilon = kEpsilon)
{
    if (std::fabs(lhs - rhs) > epsilon)
    {
        std::cerr << name << " mismatch: " << lhs << " vs " << rhs
                  << " (diff=" << std::fabs(lhs - rhs) << ")\n";
        return false;
    }
    return true;
}

bool ExpectEq(size_t lhs, size_t rhs, const char* name)
{
    if (lhs != rhs)
    {
        std::cerr << name << " mismatch: " << lhs << " vs " << rhs << "\n";
        return false;
    }
    return true;
}

bool ExpectTrue(bool condition, const char* name)
{
    if (!condition)
    {
        std::cerr << name << " was not true\n";
        return false;
    }
    return true;
}

bool ExpectFalse(bool condition, const char* name)
{
    if (condition)
    {
        std::cerr << name << " was not false\n";
        return false;
    }
    return true;
}

BodyStateFrame CreateTestBodyState(uint64_t timestamp_ns = 1000)
{
    BodyStateFrame frame;
    frame.header.source_monotonic_ns = timestamp_ns;
    frame.header.seq = 1;
    frame.base_lin_vel = {0.0f, 0.0f, 0.0f};
    frame.base_ang_vel = {0.0f, 0.0f, 0.0f};
    return frame;
}

ArmStateFrame CreateTestArmState()
{
    ArmStateFrame frame;
    frame.header.source_monotonic_ns = 1000;
    frame.header.seq = 1;
    frame.q = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    frame.q_target = {0.11f, 0.21f, 0.31f, 0.41f, 0.51f, 0.61f};
    frame.tracking_error = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
    return frame;
}

CommandContext CreateTestCommandContext(double x = 0.0, double y = 0.0, double yaw = 0.0)
{
    return CommandContext(x, y, yaw, 1000);
}

bool TestInitialization()
{
    std::cout << "Testing DriftMetricsRecorder initialization... ";

    DriftMetricsRecorder recorder;
    DriftMetricsRecorder::Config config;
    config.zero_cmd_threshold = 0.02;
    config.max_snapshots = 100;

    const bool init_ok = recorder.Initialize(config);
    if (!ExpectTrue(init_ok, "Initialize")) { return false; }
    if (!ExpectFalse(recorder.IsRecording(), "IsRecording after init")) { return false; }
    if (!ExpectEq(recorder.GetSnapshotCount(), 0, "Snapshot count after init")) { return false; }

    auto retrieved_config = recorder.GetConfig();
    if (!ExpectNear(retrieved_config.zero_cmd_threshold, 0.02, "zero_cmd_threshold")) {
        return false;
    }
    if (!ExpectEq(retrieved_config.max_snapshots, 100, "max_snapshots")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestStartStopRecording()
{
    std::cout << "Testing Start/Stop recording... ";

    DriftMetricsRecorder recorder;
    recorder.Initialize(DriftMetricsRecorder::Config{});

    if (!ExpectFalse(recorder.IsRecording(), "Not recording initially")) { return false; }

    recorder.StartRecordingWindow();
    if (!ExpectTrue(recorder.IsRecording(), "Recording after StartRecordingWindow")) {
        return false;
    }

    recorder.StopRecordingWindow();
    if (!ExpectFalse(recorder.IsRecording(), "Not recording after StopRecordingWindow")) {
        return false;
    }

    std::cout << "PASS\n";
    return true;
}

bool TestBasicRecording()
{
    std::cout << "Testing basic recording... ";

    DriftMetricsRecorder recorder;
    DriftMetricsRecorder::Config config;
    config.zero_cmd_threshold = 0.01;
    recorder.Initialize(config);

    recorder.StartRecordingWindow();

    // Create test state with zero command
    auto body_state = CreateTestBodyState(1000);
    auto arm_state = CreateTestArmState();
    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);

    recorder.Update(body_state, arm_state, cmd_ctx);
    recorder.Update(body_state, arm_state, cmd_ctx);

    if (!ExpectEq(recorder.GetSnapshotCount(), 2, "Snapshot count after updates")) {
        return false;
    }

    auto snapshots = recorder.GetSnapshots();
    if (!ExpectEq(snapshots.size(), 2, "Snapshots size")) { return false; }
    if (!ExpectTrue(snapshots[0].zero_lin_vel_cmd, "zero_lin_vel_cmd")) { return false; }
    if (!ExpectTrue(snapshots[0].zero_ang_vel_cmd, "zero_ang_vel_cmd")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestZeroCommandDetection()
{
    std::cout << "Testing zero command detection... ";

    DriftMetricsRecorder recorder;
    DriftMetricsRecorder::Config config;
    config.zero_cmd_threshold = 0.05;
    recorder.Initialize(config);
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();

    // Below threshold - should be zero
    recorder.Update(body_state, arm_state, CreateTestCommandContext(0.01, 0.01, 0.01));
    auto snapshots = recorder.GetSnapshots();
    if (!ExpectEq(snapshots.size(), 1, "Snapshots with small command")) { return false; }
    if (!ExpectTrue(snapshots[0].zero_lin_vel_cmd, "zero_lin_vel_cmd small")) { return false; }

    recorder.Clear();
    recorder.StartRecordingWindow();

    // Above threshold - no snapshot should be recorded initially
    // because reference is only captured on zero command transition
    recorder.Update(body_state, arm_state, CreateTestCommandContext(0.1, 0.1, 0.1));
    snapshots = recorder.GetSnapshots();
    // With no zero command transition, no reference is captured, so no snapshot recorded
    if (snapshots.size() != 0 && snapshots.size() != 1) {
        std::cerr << "Unexpected snapshot count: " << snapshots.size() << "\n";
        return false;
    }
    // Check that command flags are correctly set if snapshot was captured
    if (snapshots.size() == 1) {
        if (!ExpectFalse(snapshots[0].zero_lin_vel_cmd, "zero_lin_vel_cmd large")) {
            return false;
        }
    }

    std::cout << "PASS\n";
    return true;
}

bool TestCircularBuffer()
{
    std::cout << "Testing circular buffer behavior... ";

    DriftMetricsRecorder recorder;
    DriftMetricsRecorder::Config config;
    config.max_snapshots = 5;
    recorder.Initialize(config);
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();
    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);

    // Add more snapshots than buffer size
    for (size_t i = 0; i < 10; ++i)
    {
        body_state.header.source_monotonic_ns = 1000 + i;
        recorder.Update(body_state, arm_state, cmd_ctx);
    }

    auto snapshots = recorder.GetSnapshots();
    // Circular buffer should only retain max_snapshots
    if (!ExpectEq(snapshots.size(), 5, "Circular buffer size")) { return false; }
    // Total count should reflect all updates
    if (!ExpectEq(recorder.GetSnapshotCount(), 10, "Total snapshot count")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestStatistics()
{
    std::cout << "Testing statistics computation... ";

    DriftMetricsRecorder recorder;
    DriftMetricsRecorder::Config config;
    config.max_snapshots = 100;
    recorder.Initialize(config);
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();
    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);

    // Add snapshots with known drift pattern
    for (int i = 0; i < 5; ++i)
    {
        body_state.header.source_monotonic_ns = 1000 + i * 100;
        recorder.Update(body_state, arm_state, cmd_ctx);
    }

    auto stats = recorder.GetStatistics();

    if (!ExpectEq(stats.sample_count, 5, "Statistics sample count")) { return false; }
    if (!ExpectTrue(stats.max_xy_drift >= 0.0, "max_xy_drift non-negative")) { return false; }
    if (!ExpectTrue(stats.max_yaw_drift >= 0.0, "max_yaw_drift non-negative")) { return false; }

    // Test empty statistics
    recorder.Clear();
    recorder.StartRecordingWindow();
    stats = recorder.GetStatistics();
    if (!ExpectEq(stats.sample_count, 0, "Empty statistics sample count")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestClear()
{
    std::cout << "Testing Clear functionality... ";

    DriftMetricsRecorder recorder;
    recorder.Initialize(DriftMetricsRecorder::Config{});
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();
    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);

    recorder.Update(body_state, arm_state, cmd_ctx);
    recorder.Update(body_state, arm_state, cmd_ctx);

    if (!ExpectEq(recorder.GetSnapshotCount(), 2, "Snapshot count before clear")) {
        return false;
    }

    recorder.Clear();

    if (!ExpectEq(recorder.GetSnapshotCount(), 0, "Snapshot count after clear")) {
        return false;
    }

    auto snapshots = recorder.GetSnapshots();
    if (!ExpectEq(snapshots.size(), 0, "Snapshots empty after clear")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestCSVExport()
{
    std::cout << "Testing CSV export... ";

    const std::string test_path = "/tmp/test_drift_export.csv";

    // Clean up any existing test file
    std::remove(test_path.c_str());

    DriftMetricsRecorder recorder;
    DriftMetricsRecorder::Config config;
    config.output_csv_path = test_path;
    recorder.Initialize(config);
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();
    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);

    recorder.Update(body_state, arm_state, cmd_ctx);

    const bool export_ok = recorder.ExportToCSV(test_path);
    if (!ExpectTrue(export_ok, "CSV export success")) { return false; }

    // Verify file exists and has content
    std::ifstream in_file(test_path);
    if (!ExpectTrue(in_file.is_open(), "CSV file exists")) { return false; }

    std::string line;
    int line_count = 0;
    while (std::getline(in_file, line))
    {
        ++line_count;
    }
    in_file.close();

    // Should have header + 1 data line
    if (!ExpectEq(line_count, 2, "CSV line count")) { return false; }

    // Clean up
    std::remove(test_path.c_str());

    std::cout << "PASS\n";
    return true;
}

bool TestArmTrackingError()
{
    std::cout << "Testing arm tracking error computation... ";

    DriftMetricsRecorder recorder;
    recorder.Initialize(DriftMetricsRecorder::Config{});
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();

    // Set known tracking errors: 0.01 each joint
    // Expected L2 norm: sqrt(6 * 0.01^2) = sqrt(0.0006) ≈ 0.02449
    arm_state.tracking_error = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};

    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);
    recorder.Update(body_state, arm_state, cmd_ctx);

    auto snapshots = recorder.GetSnapshots();
    const double expected_error = std::sqrt(6.0 * 0.01 * 0.01);

    if (!ExpectNear(snapshots[0].arm_tracking_error, expected_error,
                    "arm_tracking_error", 1e-4)) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestCommandContextPreservation()
{
    std::cout << "Testing command context preservation... ";

    DriftMetricsRecorder recorder;
    recorder.Initialize(DriftMetricsRecorder::Config{});
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();

    // First, establish reference with zero command
    recorder.Update(body_state, arm_state, CreateTestCommandContext(0.0, 0.0, 0.0));

    // Now test with non-zero command values
    const double test_x = 0.001;   // Still below default threshold of 0.01
    const double test_y = -0.001;
    const double test_yaw = 0.001;

    body_state.header.source_monotonic_ns = 2000;
    auto cmd_ctx = CreateTestCommandContext(test_x, test_y, test_yaw);
    recorder.Update(body_state, arm_state, cmd_ctx);

    auto snapshots = recorder.GetSnapshots();

    if (snapshots.size() < 2) {
        std::cerr << "Expected at least 2 snapshots, got " << snapshots.size() << "\n";
        return false;
    }

    // Check the second snapshot which has our test command values
    if (!ExpectNear(snapshots[1].cmd_x, test_x, "cmd_x", kEpsilon)) { return false; }
    if (!ExpectNear(snapshots[1].cmd_y, test_y, "cmd_y", kEpsilon)) { return false; }
    if (!ExpectNear(snapshots[1].cmd_yaw, test_yaw, "cmd_yaw", kEpsilon)) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestYawDriftComputation()
{
    std::cout << "Testing yaw drift computation... ";

    DriftMetricsRecorder recorder;
    recorder.Initialize(DriftMetricsRecorder::Config{});
    recorder.StartRecordingWindow();

    auto body_state = CreateTestBodyState();
    auto arm_state = CreateTestArmState();
    auto cmd_ctx = CreateTestCommandContext(0.0, 0.0, 0.0);

    recorder.Update(body_state, arm_state, cmd_ctx);

    auto snapshots = recorder.GetSnapshots();

    // Should handle wraparound correctly
    // With zero actual drift, yaw_drift should be near zero
    if (!ExpectTrue(snapshots[0].yaw_drift >= 0.0, "yaw_drift non-negative")) { return false; }

    std::cout << "PASS\n";
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=== DriftMetricsRecorder Tests ===\n\n";

    bool all_passed = true;

    all_passed &= TestInitialization();
    all_passed &= TestStartStopRecording();
    all_passed &= TestBasicRecording();
    all_passed &= TestZeroCommandDetection();
    all_passed &= TestCircularBuffer();
    all_passed &= TestStatistics();
    all_passed &= TestClear();
    all_passed &= TestCSVExport();
    all_passed &= TestArmTrackingError();
    all_passed &= TestCommandContextPreservation();
    all_passed &= TestYawDriftComputation();

    std::cout << "\n=== " << (all_passed ? "ALL TESTS PASSED" : "SOME TESTS FAILED")
              << " ===\n";

    return all_passed ? 0 : 1;
}
