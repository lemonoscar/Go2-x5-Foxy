#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include "rl_sar/go2x5/state/go2_x5_state_manager.hpp"

using namespace Go2X5State;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << "\n"; \
            return 1; \
        } \
    } while(0)

#define TEST_FLOAT_EQ(a, b, msg) \
    TEST_CHECK(std::fabs((a) - (b)) < 1e-6f, msg)

int main() {
    int test_count = 0;
    int passed = 0;

    // Test 1: AtomicFlagDefaultValues
    {
        test_count++;
        StateManager manager(18, 6);

        if (!manager.IsArmBridgeConnected() &&
            !manager.IsInRLLocomotion() &&
            manager.IsArmHoldEnabled() &&
            !manager.IsArmLock() &&
            !manager.IsSafeShutdownActive()) {
            passed++;
            std::cout << "✓ AtomicFlagDefaultValues\n";
        } else {
            std::cerr << "✗ AtomicFlagDefaultValues\n";
        }
    }

    // Test 2: AtomicFlagSetAndGet
    {
        test_count++;
        StateManager manager(18, 6);
        manager.SetArmBridgeConnected(true);
        manager.SetInRLLocomotion(true);
        manager.SetArmHoldEnabled(false);
        manager.SetArmLock(true);

        if (manager.IsArmBridgeConnected() &&
            manager.IsInRLLocomotion() &&
            !manager.IsArmHoldEnabled() &&
            manager.IsArmLock()) {
            passed++;
            std::cout << "✓ AtomicFlagSetAndGet\n";
        } else {
            std::cerr << "✗ AtomicFlagSetAndGet\n";
        }
    }

    // Test 3: ConfigDefaultValues
    {
        test_count++;
        StateManager manager(18, 6);

        if (manager.GetNumDofs() == 18 &&
            manager.GetArmJointStartIndex() == 12 &&
            manager.GetArmJointCount() == 6 &&
            manager.GetArmControlMode() == "split") {
            passed++;
            std::cout << "✓ ConfigDefaultValues\n";
        } else {
            std::cerr << "✗ ConfigDefaultValues\n";
        }
    }

    // Test 4: ConfigSetAndGet
    {
        test_count++;
        StateManager manager(18, 6);
        manager.SetConfig(20, 14, 6, "bridge");

        if (manager.GetNumDofs() == 20 &&
            manager.GetArmJointStartIndex() == 14 &&
            manager.GetArmJointCount() == 6 &&
            manager.GetArmControlMode() == "bridge") {
            passed++;
            std::cout << "✓ ConfigSetAndGet\n";
        } else {
            std::cerr << "✗ ConfigSetAndGet\n";
        }
    }

    // Test 5: ArmHoldPosition
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> pos = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        manager.SetArmHoldPosition(pos);

        auto result = manager.GetArmHoldPosition();

        if (result.size() == 6u && result[0] == 0.1f && result[5] == 0.6f) {
            passed++;
            std::cout << "✓ ArmHoldPosition\n";
        } else {
            std::cerr << "✗ ArmHoldPosition\n";
        }
    }

    // Test 6: ArmCommandSmoothed
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> smoothed = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f};
        manager.SetArmCommandSmoothed(smoothed);

        auto result = manager.GetArmCommandSmoothed();

        if (result.size() == 6u && result[0] == 1.0f) {
            passed++;
            std::cout << "✓ ArmCommandSmoothed\n";
        } else {
            std::cerr << "✗ ArmCommandSmoothed\n";
        }
    }

    // Test 7: ArmJointCommandLatest
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> cmd = {0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
        manager.SetArmJointCommandLatest(cmd);

        auto result = manager.GetArmJointCommandLatest();

        bool all_match = (result.size() == cmd.size());
        for (size_t i = 0; i < cmd.size() && all_match; ++i) {
            if (result[i] != cmd[i]) all_match = false;
        }

        if (all_match) {
            passed++;
            std::cout << "✓ ArmJointCommandLatest\n";
        } else {
            std::cerr << "✗ ArmJointCommandLatest\n";
        }
    }

    // Test 8: ArmTopicCommand
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> topic_cmd = {0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f};
        manager.SetArmTopicCommandLatest(topic_cmd);

        bool has_before = manager.HasArmTopicCommand();
        manager.SetArmTopicCommandReceived(true);
        bool has_after = manager.HasArmTopicCommand();

        auto result = manager.GetArmTopicCommandLatest();

        bool all_match = (result.size() == topic_cmd.size());
        for (size_t i = 0; i < topic_cmd.size() && all_match; ++i) {
            if (result[i] != topic_cmd[i]) all_match = false;
        }

        if (!has_before && has_after && all_match) {
            passed++;
            std::cout << "✓ ArmTopicCommand\n";
        } else {
            std::cerr << "✗ ArmTopicCommand\n";
        }
    }

    // Test 9: SmoothingState
    {
        test_count++;
        StateManager manager(18, 6);
        manager.SetArmCommandSmoothingCounter(5);
        manager.SetArmCommandSmoothingTicks(100);

        std::vector<float> start = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        std::vector<float> target = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        manager.SetArmCommandSmoothingStart(start);
        manager.SetArmCommandSmoothingTarget(target);

        if (manager.GetArmCommandSmoothingCounter() == 5 &&
            manager.GetArmCommandSmoothingTicks() == 100) {
            auto start_result = manager.GetArmCommandSmoothingStart();
            auto target_result = manager.GetArmCommandSmoothingTarget();

            bool all_match = (start_result.size() == start.size() &&
                             target_result.size() == target.size());
            for (size_t i = 0; i < start.size() && all_match; ++i) {
                if (start_result[i] != start[i] || target_result[i] != target[i]) {
                    all_match = false;
                }
            }

            if (all_match) {
                passed++;
                std::cout << "✓ SmoothingState\n";
            } else {
                std::cerr << "✗ SmoothingState\n";
            }
        } else {
            std::cerr << "✗ SmoothingState\n";
        }
    }

    // Test 10: ArmBridgeState
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> q = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
        std::vector<float> tau = {0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};

        manager.SetArmBridgeState(q, dq, tau, true);

        if (manager.IsArmBridgeStateValid()) {
            std::vector<float> q_out, dq_out, tau_out;
            manager.GetArmBridgeState(&q_out, &dq_out, &tau_out);

            bool all_match = (q_out.size() == q.size() &&
                             dq_out.size() == dq.size() &&
                             tau_out.size() == tau.size());
            for (size_t i = 0; i < q.size() && all_match; ++i) {
                if (q_out[i] != q[i] || dq_out[i] != dq[i] || tau_out[i] != tau[i]) {
                    all_match = false;
                }
            }

            if (all_match) {
                passed++;
                std::cout << "✓ ArmBridgeState\n";
            } else {
                std::cerr << "✗ ArmBridgeState\n";
            }
        } else {
            std::cerr << "✗ ArmBridgeState\n";
        }
    }

    // Test 11: ArmBridgeStateFreshness
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> q = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
        std::vector<float> tau = {0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};

        manager.SetArmBridgeState(q, dq, tau, true);

        bool is_fresh = manager.IsArmBridgeStateFresh(250.0);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        bool is_stale = !manager.IsArmBridgeStateFresh(250.0);

        if (is_fresh && is_stale) {
            passed++;
            std::cout << "✓ ArmBridgeStateFreshness\n";
        } else {
            std::cerr << "✗ ArmBridgeStateFreshness\n";
        }
    }

    // Test 12: ArmBridgeStateInvalidByDefault
    {
        test_count++;
        StateManager manager(18, 6);

        if (!manager.IsArmBridgeStateValid()) {
            std::vector<float> q_out, dq_out, tau_out;
            if (!manager.GetArmBridgeState(&q_out, &dq_out, &tau_out)) {
                passed++;
                std::cout << "✓ ArmBridgeStateInvalidByDefault\n";
            } else {
                std::cerr << "✗ ArmBridgeStateInvalidByDefault\n";
            }
        } else {
            std::cerr << "✗ ArmBridgeStateInvalidByDefault\n";
        }
    }

    // Test 13: CaptureSnapshot
    {
        test_count++;
        StateManager manager(18, 6);
        manager.SetArmBridgeConnected(true);
        manager.SetInRLLocomotion(true);
        manager.SetArmHoldEnabled(false);
        manager.SetArmLock(true);

        std::vector<float> pos = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        manager.SetArmHoldPosition(pos);

        auto snapshot = manager.CaptureSnapshot();

        bool all_match = (snapshot.arm_bridge_connected &&
                          snapshot.in_rl_locomotion &&
                          !snapshot.arm_hold_enabled &&
                          snapshot.arm_lock &&
                          snapshot.arm_hold_position.size() == pos.size());
        for (size_t i = 0; i < pos.size() && all_match; ++i) {
            if (snapshot.arm_hold_position[i] != pos[i]) all_match = false;
        }

        if (all_match) {
            passed++;
            std::cout << "✓ CaptureSnapshot\n";
        } else {
            std::cerr << "✗ CaptureSnapshot\n";
        }
    }

    // Test 14: RestoreFromSnapshot
    {
        test_count++;
        StateManager manager(18, 6);

        StateSnapshot snapshot;
        snapshot.arm_hold_enabled = false;
        snapshot.arm_lock = true;
        snapshot.arm_hold_position = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};

        manager.RestoreFromSnapshot(snapshot);

        auto pos = manager.GetArmHoldPosition();

        bool pos_match = (pos.size() == snapshot.arm_hold_position.size());
        for (size_t i = 0; i < snapshot.arm_hold_position.size() && pos_match; ++i) {
            if (pos[i] != snapshot.arm_hold_position[i]) pos_match = false;
        }

        if (pos_match) {
            passed++;
            std::cout << "✓ RestoreFromSnapshot\n";
        } else {
            std::cerr << "✗ RestoreFromSnapshot\n";
        }
    }

    // Test 15: ResetClearsAllState
    {
        test_count++;
        StateManager manager(18, 6);

        manager.SetArmBridgeConnected(true);
        manager.SetInRLLocomotion(true);
        manager.SetArmHoldEnabled(false);
        manager.SetArmLock(true);

        manager.Reset();

        if (!manager.IsArmBridgeConnected() &&
            !manager.IsInRLLocomotion() &&
            manager.IsArmHoldEnabled() &&
            !manager.IsArmLock() &&
            !manager.IsSafeShutdownActive()) {
            passed++;
            std::cout << "✓ ResetClearsAllState\n";
        } else {
            std::cerr << "✗ ResetClearsAllState\n";
        }
    }

    // Test 16: ResetArmCommandState
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> pos = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        manager.SetArmHoldPosition(pos);
        manager.SetArmTopicCommandReceived(true);

        manager.ResetArmCommandState();

        auto result = manager.GetArmHoldPosition();

        bool all_zero = true;
        for (float val : result) {
            if (val != 0.0f) all_zero = false;
        }

        if (all_zero && !manager.HasArmTopicCommand()) {
            passed++;
            std::cout << "✓ ResetArmCommandState\n";
        } else {
            std::cerr << "✗ ResetArmCommandState\n";
        }
    }

    // Test 17: ResetArmBridgeState
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> q = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        std::vector<float> dq = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
        std::vector<float> tau = {0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};

        manager.SetArmBridgeState(q, dq, tau, true);

        bool was_valid = manager.IsArmBridgeStateValid();

        manager.ResetArmBridgeState();

        if (was_valid && !manager.IsArmBridgeStateValid()) {
            passed++;
            std::cout << "✓ ResetArmBridgeState\n";
        } else {
            std::cerr << "✗ ResetArmBridgeState\n";
        }
    }

    // Test 18: ArmCommandIgnoresWrongSize
    {
        test_count++;
        StateManager manager(18, 6);
        std::vector<float> wrong_size = {0.1f, 0.2f};
        manager.SetArmHoldPosition(wrong_size);

        auto result = manager.GetArmHoldPosition();

        if (result.size() == 6u) {
            passed++;
            std::cout << "✓ ArmCommandIgnoresWrongSize\n";
        } else {
            std::cerr << "✗ ArmCommandIgnoresWrongSize\n";
        }
    }

    // Test 21: ConcurrentArmCommandAccess
    {
        test_count++;
        StateManager manager(18, 6);

        const int num_threads = 10;
        const int iterations = 100;
        std::vector<std::thread> threads;

        for (int t = 0; t < num_threads; ++t) {
            threads.emplace_back([&manager, iterations]() {
                for (int i = 0; i < iterations; ++i) {
                    std::vector<float> pos = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
                    manager.SetArmHoldPosition(pos);
                    auto result = manager.GetArmHoldPosition();
                    (void)result.size();
                }
            });
        }

        for (auto& thread : threads) {
            thread.join();
        }

        passed++;
        std::cout << "✓ ConcurrentArmCommandAccess\n";
    }

    // Test 22: ScopedStateRead
    {
        test_count++;
        StateManager mgr(18, 6);

        std::vector<float> pos = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        mgr.SetArmHoldPosition(pos);
        mgr.SetArmBridgeConnected(true);

        ScopedStateRead read(mgr);
        const auto& snapshot = read.Snapshot();

        bool all_match = (snapshot.arm_joint_count == 6 &&
                          snapshot.arm_hold_position.size() == pos.size());
        for (size_t i = 0; i < pos.size() && all_match; ++i) {
            if (snapshot.arm_hold_position[i] != pos[i]) all_match = false;
        }

        if (all_match && snapshot.arm_bridge_connected) {
            passed++;
            std::cout << "✓ ScopedStateRead\n";
        } else {
            std::cerr << "✗ ScopedStateRead\n";
        }
    }

    std::cout << "\n========================\n";
    std::cout << "Test Results: " << passed << "/" << test_count << " passed\n";

    return (passed == test_count) ? 0 : 1;
}
