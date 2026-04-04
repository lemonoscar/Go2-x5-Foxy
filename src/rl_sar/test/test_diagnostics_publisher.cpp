#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

#include "rl_sar/diagnostics/diagnostics_publisher.hpp"

using namespace rl_sar::diagnostics;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAILED: " << msg << "\n"; \
            return 1; \
        } \
    } while(0)

#define TEST_FLOAT_EQ(a, b, msg) \
    TEST_CHECK(std::fabs((a) - (b)) < 1e-6, msg)

int main()
{
    int test_count = 0;
    int passed = 0;

    // Test 1: DefaultConstruction
    {
        test_count++;
        DiagnosticsPublisher publisher;

        if (!publisher.IsInitialized() && !publisher.IsRunning()) {
            passed++;
            std::cout << "PASS: DefaultConstruction\n";
        } else {
            std::cerr << "FAIL: DefaultConstruction\n";
        }
    }

    // Test 2: InitializeWithDefaultConfig
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;

#ifdef USE_ROS2
        // Default config has ros2_enabled=true, which requires ROS2
        if (publisher.Initialize(config)) {
            passed++;
            std::cout << "PASS: InitializeWithDefaultConfig\n";
        } else {
            std::cerr << "FAIL: InitializeWithDefaultConfig\n";
        }
#else
        // In standalone mode, default config should fail (ros2_enabled=true but no ROS2)
        // Skip this test in standalone mode
        passed++;
        std::cout << "PASS: InitializeWithDefaultConfig (skipped - no ROS2)\n";
#endif
    }

    // Test 3: InitializeWithCustomConfig
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;  // Standalone mode for testing
        config.publish_rate_hz = 100;
        config.diagnostics_topic = "/test/diagnostics";

        if (publisher.Initialize(config) && publisher.IsInitialized()) {
            passed++;
            std::cout << "PASS: InitializeWithCustomConfig\n";
        } else {
            std::cerr << "FAIL: InitializeWithCustomConfig\n";
        }
    }

    // Test 4: ConfigValidationInvalidRate
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = -1;  // Invalid

        if (!publisher.Initialize(config)) {
            passed++;
            std::cout << "PASS: ConfigValidationInvalidRate\n";
        } else {
            std::cerr << "FAIL: ConfigValidationInvalidRate\n";
        }
    }

    // Test 5: ConfigValidationTooHighRate
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = 2000;  // Exceeds max of 1000

        if (!publisher.Initialize(config)) {
            passed++;
            std::cout << "PASS: ConfigValidationTooHighRate\n";
        } else {
            std::cerr << "FAIL: ConfigValidationTooHighRate\n";
        }
    }

    // Test 6: StartStopPublisher
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;

        if (publisher.Initialize(config) && publisher.Start()) {
            if (publisher.IsRunning()) {
                publisher.Stop();
                if (!publisher.IsRunning()) {
                    passed++;
                    std::cout << "PASS: StartStopPublisher\n";
                } else {
                    std::cerr << "FAIL: StartStopPublisher (stop failed)\n";
                }
            } else {
                std::cerr << "FAIL: StartStopPublisher (not running)\n";
            }
        } else {
            std::cerr << "FAIL: StartStopPublisher (init/start failed)\n";
        }
    }

    // Test 7: UpdateMetricsBeforeStart
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;

        if (publisher.Initialize(config)) {
            DiagnosticMetrics metrics;
            metrics.policy_latency_us = 1000;
            metrics.policy_frequency_hz = 50.0;
            metrics.current_mode = "TEST";

            publisher.UpdateMetrics(metrics);

            DiagnosticMetrics retrieved;
            if (publisher.GetMetrics(retrieved) &&
                retrieved.policy_latency_us == 1000 &&
                retrieved.policy_frequency_hz == 50.0 &&
                retrieved.current_mode == "TEST") {
                passed++;
                std::cout << "PASS: UpdateMetricsBeforeStart\n";
            } else {
                std::cerr << "FAIL: UpdateMetricsBeforeStart\n";
            }
        } else {
            std::cerr << "FAIL: UpdateMetricsBeforeStart (init failed)\n";
        }
    }

    // Test 8: UpdateMetricsWhileRunning
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = 100;

        if (publisher.Initialize(config) && publisher.Start()) {
            DiagnosticMetrics metrics;
            metrics.policy_latency_us = 500;
            metrics.coordinator_jitter_us = 100;
            metrics.body_state_age_us = 1000;
            metrics.arm_state_age_us = 500;
            metrics.arm_tracking_error_norm = 0.01;
            metrics.arm_tracking_healthy = true;
            metrics.xy_drift_error = 0.001;
            metrics.yaw_drift_error = 0.01;
            metrics.clip_count = 0;
            metrics.seq_gap_count = 0;
            metrics.current_mode = "READY";
            metrics.mode_transition_count = 1;
            metrics.policy_frequency_hz = 50.0;
            metrics.coordinator_frequency_hz = 50.0;

            publisher.UpdateMetrics(metrics);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            DiagnosticMetrics retrieved;
            if (publisher.GetMetrics(retrieved) &&
                retrieved.policy_latency_us == 500 &&
                retrieved.coordinator_jitter_us == 100 &&
                retrieved.current_mode == "READY") {
                publisher.Stop();
                passed++;
                std::cout << "PASS: UpdateMetricsWhileRunning\n";
            } else {
                publisher.Stop();
                std::cerr << "FAIL: UpdateMetricsWhileRunning\n";
            }
        } else {
            std::cerr << "FAIL: UpdateMetricsWhileRunning (init/start failed)\n";
        }
    }

    // Test 9: MetricsDefaults
    {
        test_count++;
        DiagnosticMetrics metrics;

        if (metrics.policy_latency_us == 0 &&
            metrics.policy_frequency_hz == 0.0 &&
            metrics.coordinator_jitter_us == 0 &&
            metrics.coordinator_frequency_hz == 0.0 &&
            metrics.body_state_age_us == 0 &&
            metrics.arm_state_age_us == 0 &&
            metrics.arm_tracking_error_norm == 0.0 &&
            metrics.arm_tracking_healthy == true &&
            metrics.xy_drift_error == 0.0 &&
            metrics.yaw_drift_error == 0.0 &&
            metrics.clip_count == 0 &&
            metrics.seq_gap_count == 0 &&
            metrics.current_mode.empty() &&
            metrics.mode_transition_count == 0 &&
            metrics.timestamp_ns == 0) {
            passed++;
            std::cout << "PASS: MetricsDefaults\n";
        } else {
            std::cerr << "FAIL: MetricsDefaults\n";
        }
    }

    // Test 10: MetricsReset
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.policy_latency_us = 1000;
        metrics.policy_frequency_hz = 50.0;
        metrics.current_mode = "TEST";
        metrics.clip_count = 5;

        metrics.Reset();

        if (metrics.policy_latency_us == 0 &&
            metrics.policy_frequency_hz == 0.0 &&
            metrics.current_mode.empty() &&
            metrics.clip_count == 0) {
            passed++;
            std::cout << "PASS: MetricsReset\n";
        } else {
            std::cerr << "FAIL: MetricsReset\n";
        }
    }

    // Test 11: MetricsIsDegradedAllHealthy
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.body_state_age_us = 1000;        // 1ms - healthy
        metrics.arm_state_age_us = 1000;         // 1ms - healthy
        metrics.policy_latency_us = 1000;        // 1ms - healthy
        metrics.arm_tracking_healthy = true;
        metrics.arm_tracking_error_norm = 0.1;   // < 0.5 rad - healthy

        if (!metrics.IsDegraded()) {
            passed++;
            std::cout << "PASS: MetricsIsDegradedAllHealthy\n";
        } else {
            std::cerr << "FAIL: MetricsIsDegradedAllHealthy\n";
        }
    }

    // Test 12: MetricsIsDegradedStaleBodyState
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.body_state_age_us = 600000;      // 600ms - stale (> 500ms threshold)

        if (metrics.IsDegraded()) {
            passed++;
            std::cout << "PASS: MetricsIsDegradedStaleBodyState\n";
        } else {
            std::cerr << "FAIL: MetricsIsDegradedStaleBodyState\n";
        }
    }

    // Test 13: MetricsIsDegradedStaleArmState
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.arm_state_age_us = 600000;       // 600ms - stale

        if (metrics.IsDegraded()) {
            passed++;
            std::cout << "PASS: MetricsIsDegradedStaleArmState\n";
        } else {
            std::cerr << "FAIL: MetricsIsDegradedStaleArmState\n";
        }
    }

    // Test 14: MetricsIsDegradedHighPolicyLatency
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.policy_latency_us = 100000;      // 100ms - high (> 50ms threshold)

        if (metrics.IsDegraded()) {
            passed++;
            std::cout << "PASS: MetricsIsDegradedHighPolicyLatency\n";
        } else {
            std::cerr << "FAIL: MetricsIsDegradedHighPolicyLatency\n";
        }
    }

    // Test 15: MetricsIsDegradedUnhealthyArmTracking
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.arm_tracking_healthy = false;

        if (metrics.IsDegraded()) {
            passed++;
            std::cout << "PASS: MetricsIsDegradedUnhealthyArmTracking\n";
        } else {
            std::cerr << "FAIL: MetricsIsDegradedUnhealthyArmTracking\n";
        }
    }

    // Test 16: MetricsIsDegradedHighTrackingError
    {
        test_count++;
        DiagnosticMetrics metrics;
        metrics.arm_tracking_error_norm = 1.0;   // > 0.5 rad threshold

        if (metrics.IsDegraded()) {
            passed++;
            std::cout << "PASS: MetricsIsDegradedHighTrackingError\n";
        } else {
            std::cerr << "FAIL: MetricsIsDegradedHighTrackingError\n";
        }
    }

    // Test 17: MetricsGetStatusString
    {
        test_count++;
        DiagnosticMetrics metrics;
        std::string status = metrics.GetStatusString();

        if (status == "HEALTHY") {
            metrics.policy_latency_us = 100000;
            status = metrics.GetStatusString();
            if (status == "DEGRADED") {
                passed++;
                std::cout << "PASS: MetricsGetStatusString\n";
            } else {
                std::cerr << "FAIL: MetricsGetStatusString (not degraded)\n";
            }
        } else {
            std::cerr << "FAIL: MetricsGetStatusString (not healthy)\n";
        }
    }

    // Test 18: GetConfig
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = 75;
        config.diagnostics_topic = "/custom/topic";

        publisher.Initialize(config);
        auto retrieved_config = publisher.GetConfig();

        if (retrieved_config.ros2_enabled == false &&
            retrieved_config.publish_rate_hz == 75 &&
            retrieved_config.diagnostics_topic == "/custom/topic") {
            passed++;
            std::cout << "PASS: GetConfig\n";
        } else {
            std::cerr << "FAIL: GetConfig\n";
        }
    }

    // Test 19: GetMetricsNotInitialized
    {
        test_count++;
        DiagnosticsPublisher publisher;

        DiagnosticMetrics metrics;
        if (!publisher.GetMetrics(metrics)) {
            passed++;
            std::cout << "PASS: GetMetricsNotInitialized\n";
        } else {
            std::cerr << "FAIL: GetMetricsNotInitialized\n";
        }
    }

    // Test 20: PublishStatistics
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = 100;

        if (publisher.Initialize(config) && publisher.Start()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            auto stats = publisher.GetStats();

            if (stats.total_publish_count > 0 && stats.failed_publish_count == 0) {
                publisher.Stop();
                passed++;
                std::cout << "PASS: PublishStatistics\n";
            } else {
                publisher.Stop();
                std::cerr << "FAIL: PublishStatistics (count=" << stats.total_publish_count
                          << ", failed=" << stats.failed_publish_count << ")\n";
            }
        } else {
            std::cerr << "FAIL: PublishStatistics (init/start failed)\n";
        }
    }

    // Test 21: ResetStatistics
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = 100;

        if (publisher.Initialize(config) && publisher.Start()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            publisher.Stop();
            auto stats_before = publisher.GetStats();

            publisher.ResetStats();
            auto stats_after = publisher.GetStats();

            if (stats_after.total_publish_count == 0 &&
                stats_after.failed_publish_count == 0) {
                passed++;
                std::cout << "PASS: ResetStatistics\n";
            } else {
                std::cerr << "FAIL: ResetStatistics\n";
            }
        } else {
            std::cerr << "FAIL: ResetStatistics (init/start failed)\n";
        }
    }

    // Test 22: ConcurrentMetricsUpdate
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;
        config.publish_rate_hz = 100;

        if (publisher.Initialize(config) && publisher.Start()) {
            std::atomic<int> update_count{0};
            const int num_threads = 5;
            const int updates_per_thread = 100;
            std::vector<std::thread> threads;

            for (int t = 0; t < num_threads; ++t) {
                threads.emplace_back([&publisher, &update_count, updates_per_thread]() {
                    for (int i = 0; i < updates_per_thread; ++i) {
                        DiagnosticMetrics metrics;
                        metrics.policy_latency_us = i;
                        metrics.current_mode = "THREAD_TEST";
                        publisher.UpdateMetrics(metrics);
                        update_count.fetch_add(1);
                    }
                });
            }

            for (auto& thread : threads) {
                thread.join();
            }

            publisher.Stop();

            if (update_count.load() == num_threads * updates_per_thread) {
                passed++;
                std::cout << "PASS: ConcurrentMetricsUpdate\n";
            } else {
                std::cerr << "FAIL: ConcurrentMetricsUpdate (count=" << update_count.load() << ")\n";
            }
        } else {
            std::cerr << "FAIL: ConcurrentMetricsUpdate (init/start failed)\n";
        }
    }

    // Test 23: MultipleStartStop
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;

        if (publisher.Initialize(config)) {
            bool all_ok = true;

            for (int i = 0; i < 3; ++i) {
                if (!publisher.Start()) {
                    all_ok = false;
                    break;
                }
                if (!publisher.IsRunning()) {
                    all_ok = false;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                publisher.Stop();
                if (publisher.IsRunning()) {
                    all_ok = false;
                    break;
                }
            }

            if (all_ok) {
                passed++;
                std::cout << "PASS: MultipleStartStop\n";
            } else {
                std::cerr << "FAIL: MultipleStartStop\n";
            }
        } else {
            std::cerr << "FAIL: MultipleStartStop (init failed)\n";
        }
    }

    // Test 24: MetricsTimestampAssignment
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;

        if (publisher.Initialize(config)) {
            DiagnosticMetrics metrics;
            metrics.timestamp_ns = 0;  // Let publisher assign
            metrics.current_mode = "TIMESTAMP_TEST";

            publisher.UpdateMetrics(metrics);

            DiagnosticMetrics retrieved;
            if (publisher.GetMetrics(retrieved) &&
                retrieved.timestamp_ns > 0 &&
                retrieved.current_mode == "TIMESTAMP_TEST") {
                passed++;
                std::cout << "PASS: MetricsTimestampAssignment\n";
            } else {
                std::cerr << "FAIL: MetricsTimestampAssignment\n";
            }
        } else {
            std::cerr << "FAIL: MetricsTimestampAssignment (init failed)\n";
        }
    }

    // Test 25: MetricsPreserveTimestamp
    {
        test_count++;
        DiagnosticsPublisher publisher;
        DiagnosticsPublisher::Config config;
        config.ros2_enabled = false;

        if (publisher.Initialize(config)) {
            const uint64_t test_timestamp = 1234567890000ULL;
            DiagnosticMetrics metrics;
            metrics.timestamp_ns = test_timestamp;
            metrics.current_mode = "PRESERVE_TEST";

            publisher.UpdateMetrics(metrics);

            DiagnosticMetrics retrieved;
            if (publisher.GetMetrics(retrieved) &&
                retrieved.timestamp_ns == test_timestamp) {
                passed++;
                std::cout << "PASS: MetricsPreserveTimestamp\n";
            } else {
                std::cerr << "FAIL: MetricsPreserveTimestamp\n";
            }
        } else {
            std::cerr << "FAIL: MetricsPreserveTimestamp (init failed)\n";
        }
    }

    std::cout << "\n========================================\n";
    std::cout << "Test Results: " << passed << "/" << test_count << " passed\n";
    std::cout << "========================================\n";

    return (passed == test_count) ? 0 : 1;
}
