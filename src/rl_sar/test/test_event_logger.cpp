#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstring>

#include "rl_sar/logger/event_logger.hpp"

namespace
{

using namespace rl_sar::logger;

constexpr double kEpsilon = 1e-6;

bool ExpectEq(size_t lhs, size_t rhs, const char* name)
{
    if (lhs != rhs)
    {
        std::cerr << name << " mismatch: " << lhs << " vs " << rhs << "\n";
        return false;
    }
    return true;
}

bool ExpectStrEq(const std::string& lhs, const std::string& rhs, const char* name)
{
    if (lhs != rhs)
    {
        std::cerr << name << " mismatch: \"" << lhs << "\" vs \"" << rhs << "\"\n";
        return false;
    }
    return true;
}

bool ExpectStrEq(const char* lhs, const char* rhs, const char* name)
{
    if (std::strcmp(lhs, rhs) != 0)
    {
        std::cerr << name << " mismatch: \"" << lhs << "\" vs \"" << rhs << "\"\n";
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

std::string ReadFileContents(const std::string& path)
{
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

bool TestInitializeSuccess()
{
    std::cout << "Testing EventLogger initialization... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_file_logging = true;
    config.enable_console_logging = false;
    config.enable_json_logging = true;
    config.log_file_path = "/tmp/test_event_logger_init.log";
    config.json_log_path = "/tmp/test_event_logger_init.jsonl";

    // Clean up any existing test files
    std::remove(config.log_file_path.c_str());
    std::remove(config.json_log_path.c_str());

    const bool init_ok = logger.Initialize(config);
    if (!ExpectTrue(init_ok, "Initialize")) { return false; }
    if (!ExpectTrue(logger.IsInitialized(), "IsInitialized")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestModeToString()
{
    std::cout << "Testing ModeToString... ";

    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::BOOT), "BOOT", "BOOT")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::PROBE), "PROBE", "PROBE")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::PASSIVE), "PASSIVE", "PASSIVE")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::READY), "READY", "READY")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::RL_DOG_ONLY_ACTIVE), "RL_DOG_ONLY_ACTIVE", "RL_DOG_ONLY_ACTIVE")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::MANUAL_ARM), "MANUAL_ARM", "MANUAL_ARM")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::DEGRADED_ARM), "DEGRADED_ARM", "DEGRADED_ARM")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::DEGRADED_BODY), "DEGRADED_BODY", "DEGRADED_BODY")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::SOFT_STOP), "SOFT_STOP", "SOFT_STOP")) { return false; }
    if (!ExpectStrEq(EventLogger::ModeToString(ModeType::FAULT_LATCHED), "FAULT_LATCHED", "FAULT_LATCHED")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestFaultToString()
{
    std::cout << "Testing FaultToString... ";

    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::BODY_STATE_STALE), "BODY_STATE_STALE", "BODY_STATE_STALE")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::ARM_STATE_STALE), "ARM_STATE_STALE", "ARM_STATE_STALE")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::POLICY_STALE), "POLICY_STALE", "POLICY_STALE")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::ARM_TRACKING_ERROR), "ARM_TRACKING_ERROR", "ARM_TRACKING_ERROR")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::SAFETY_LIMIT_EXCEEDED), "SAFETY_LIMIT_EXCEEDED", "SAFETY_LIMIT_EXCEEDED")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::DDS_FAILURE), "DDS_FAILURE", "DDS_FAILURE")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::CAN_FAILURE), "CAN_FAILURE", "CAN_FAILURE")) { return false; }
    if (!ExpectStrEq(EventLogger::FaultToString(FaultType::UNKNOWN), "UNKNOWN", "UNKNOWN")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestLogBasicModeTransition()
{
    std::cout << "Testing basic mode transition logging... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;
    config.log_file_path = "/tmp/test_mode_transition.log";
    config.json_log_path = "/tmp/test_mode_transition.jsonl";
    std::remove(config.log_file_path.c_str());
    std::remove(config.json_log_path.c_str());

    logger.Initialize(config);

    logger.LogModeTransition(ModeType::READY, ModeType::RL_DOG_ONLY_ACTIVE,
                              "Operator enabled");

    auto events = logger.GetEvents();

    // Find the transition event (skip init event)
    const EventLogEntry* transition = nullptr;
    for (const auto& e : events)
    {
        if (e.from_mode == ModeType::READY && e.to_mode == ModeType::RL_DOG_ONLY_ACTIVE)
        {
            transition = &e;
            break;
        }
    }

    if (!ExpectTrue(transition != nullptr, "Found transition event")) { return false; }
    if (!ExpectFalse(transition->is_fault, "Not a fault")) { return false; }
    if (!ExpectStrEq(transition->reason, std::string("Operator enabled"), "Reason")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestLogModeTransitionWithContext()
{
    std::cout << "Testing mode transition with context... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;
    config.log_file_path = "/tmp/test_mode_context.log";
    config.json_log_path = "/tmp/test_mode_context.jsonl";
    std::remove(config.log_file_path.c_str());
    std::remove(config.json_log_path.c_str());

    logger.Initialize(config);

    logger.LogModeTransition(
        ModeType::PASSIVE,
        ModeType::READY,
        "System ready",
        10.5,   // body_state_age_us
        15.2,   // arm_state_age_us
        0.001,  // arm_tracking_error
        "policy_v1");

    auto events = logger.GetEvents();

    const EventLogEntry* transition = nullptr;
    for (const auto& e : events)
    {
        if (e.from_mode == ModeType::PASSIVE && e.to_mode == ModeType::READY)
        {
            transition = &e;
            break;
        }
    }

    if (!ExpectTrue(transition != nullptr, "Found transition event")) { return false; }
    if (!ExpectNear(transition->body_state_age_us, 10.5, "body_state_age_us")) { return false; }
    if (!ExpectNear(transition->arm_state_age_us, 15.2, "arm_state_age_us")) { return false; }
    if (!ExpectNear(transition->arm_tracking_error, 0.001, "arm_tracking_error")) { return false; }
    if (!ExpectStrEq(transition->current_policy_id, std::string("policy_v1"), "policy_id")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestLogBasicFault()
{
    std::cout << "Testing basic fault logging... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;
    config.log_file_path = "/tmp/test_fault.log";
    config.json_log_path = "/tmp/test_fault.jsonl";
    std::remove(config.log_file_path.c_str());
    std::remove(config.json_log_path.c_str());

    logger.Initialize(config);

    logger.LogFault(FaultType::BODY_STATE_STALE, "Body state timeout");

    auto events = logger.GetEvents();

    const EventLogEntry* fault = nullptr;
    for (const auto& e : events)
    {
        if (e.is_fault && e.fault_type == FaultType::BODY_STATE_STALE)
        {
            fault = &e;
            break;
        }
    }

    if (!ExpectTrue(fault != nullptr, "Found fault event")) { return false; }
    if (!ExpectTrue(fault->is_fault, "Is a fault")) { return false; }
    if (!ExpectTrue(fault->fault_type == FaultType::BODY_STATE_STALE, "fault_type")) { return false; }
    if (!ExpectStrEq(fault->fault_details, std::string("Body state timeout"), "fault_details")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestGetFaultEvents()
{
    std::cout << "Testing GetFaultEvents... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;

    logger.Initialize(config);

    logger.LogModeTransition(ModeType::BOOT, ModeType::PROBE, "Boot complete");
    logger.LogFault(FaultType::BODY_STATE_STALE, "Stale body state");
    logger.LogFault(FaultType::ARM_STATE_STALE, "Stale arm state");

    auto faults = logger.GetFaultEvents();

    if (!ExpectEq(faults.size(), 2u, "Fault count")) { return false; }
    if (!ExpectTrue(faults[0].is_fault, "First is fault")) { return false; }
    if (!ExpectTrue(faults[1].is_fault, "Second is fault")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestGetEventsByMode()
{
    std::cout << "Testing GetEventsByMode... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;

    logger.Initialize(config);

    logger.LogModeTransition(ModeType::READY, ModeType::RL_DOG_ONLY_ACTIVE, "Enable");
    logger.LogModeTransition(ModeType::RL_DOG_ONLY_ACTIVE, ModeType::DEGRADED_ARM, "Fault");
    logger.LogModeTransition(ModeType::DEGRADED_ARM, ModeType::READY, "Recovered");

    auto ready_events = logger.GetEventsByMode(ModeType::READY);

    // Should find transitions involving READY mode (READY->ACTIVE and DEGRADED->READY)
    // At minimum, should have 2 events
    if (!ExpectTrue(ready_events.size() >= 2u, "READY mode events count")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestClearEvents()
{
    std::cout << "Testing Clear events... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;

    logger.Initialize(config);

    logger.LogModeTransition(ModeType::BOOT, ModeType::PROBE, "Test");
    if (!ExpectTrue(logger.GetEventCount() >= 1u, "Event count before clear")) { return false; }

    logger.Clear();
    if (!ExpectEq(logger.GetEventCount(), 0u, "Event count after clear")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestSequenceNumbers()
{
    std::cout << "Testing sequence numbers... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;

    logger.Initialize(config);

    logger.LogModeTransition(ModeType::BOOT, ModeType::PROBE, "Step 1");
    logger.LogModeTransition(ModeType::PROBE, ModeType::PASSIVE, "Step 2");

    auto events = logger.GetEvents();

    // Find the two transitions
    uint64_t seq1 = 0, seq2 = 0;
    for (const auto& e : events)
    {
        if (e.reason == "Step 1") seq1 = e.seq;
        if (e.reason == "Step 2") seq2 = e.seq;
    }

    if (!ExpectTrue(seq2 > seq1, "Sequence numbers increment")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestCircularBuffer()
{
    std::cout << "Testing circular buffer... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;
    config.max_memory_entries = 5;

    logger.Initialize(config);

    // Log more events than buffer size
    for (int i = 0; i < 10; ++i)
    {
        logger.LogModeTransition(ModeType::BOOT, ModeType::PROBE,
                                  "Event " + std::to_string(i));
    }

    // Buffer should not exceed max size
    if (!ExpectTrue(logger.GetEventCount() <= 5u, "Buffer size within limit")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestJsonOutput()
{
    std::cout << "Testing JSON output format... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;
    config.enable_json_logging = true;
    config.log_file_path = "/tmp/test_json.log";
    config.json_log_path = "/tmp/test_json_output.jsonl";
    std::remove(config.log_file_path.c_str());
    std::remove(config.json_log_path.c_str());

    logger.Initialize(config);
    logger.LogFault(FaultType::BODY_STATE_STALE, "Test fault");
    logger.Flush();

    std::string content = ReadFileContents(config.json_log_path);

    // Check for expected JSON fields
    if (!ExpectTrue(content.find("\"ts_ns\":") != std::string::npos, "JSON has ts_ns")) { return false; }
    if (!ExpectTrue(content.find("\"seq\":") != std::string::npos, "JSON has seq")) { return false; }
    if (!ExpectTrue(content.find("\"is_fault\":true") != std::string::npos, "JSON has is_fault")) { return false; }
    if (!ExpectTrue(content.find("\"fault_type\":\"BODY_STATE_STALE\"") != std::string::npos, "JSON has fault_type")) { return false; }
    if (!ExpectTrue(content.find("\"fault_details\":\"Test fault\"") != std::string::npos, "JSON has fault_details")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestConcurrentLogging()
{
    std::cout << "Testing concurrent logging... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;

    logger.Initialize(config);

    const int num_threads = 4;
    const int events_per_thread = 25;
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t)
    {
        threads.emplace_back([&logger, t, events_per_thread]() {
            for (int i = 0; i < events_per_thread; ++i)
            {
                logger.LogModeTransition(
                    ModeType::BOOT,
                    ModeType::PROBE,
                    "Thread " + std::to_string(t) + " event " + std::to_string(i));
            }
        });
    }

    for (auto& thread : threads)
    {
        thread.join();
    }

    // All events should be logged (accounting for circular buffer and init event)
    auto events = logger.GetEvents();
    // The buffer may not hold all events if max_memory_entries is small
    // Default is 1000, so we should have at least num_threads * events_per_thread + init
    if (!ExpectTrue(events.size() >= static_cast<size_t>(num_threads * events_per_thread),
                    "Event count after concurrent logging")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestJsonStringEscape()
{
    std::cout << "Testing JSON string escaping... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;
    config.enable_json_logging = true;
    config.log_file_path = "/tmp/test_escape.log";
    config.json_log_path = "/tmp/test_escape.jsonl";
    std::remove(config.log_file_path.c_str());
    std::remove(config.json_log_path.c_str());

    logger.Initialize(config);

    logger.LogFault(FaultType::UNKNOWN,
                     "Test with \"quotes\" and \\backslashes\n and newlines");

    logger.Flush();

    std::string content = ReadFileContents(config.json_log_path);

    // Check that special characters are escaped
    if (!ExpectTrue(content.find("\\\"quotes\\\"") != std::string::npos, "Quotes escaped")) { return false; }
    if (!ExpectTrue(content.find("\\\\backslashes") != std::string::npos, "Backslashes escaped")) { return false; }
    if (!ExpectTrue(content.find("\\n") != std::string::npos, "Newlines escaped")) { return false; }

    std::cout << "PASS\n";
    return true;
}

bool TestCustomEvent()
{
    std::cout << "Testing custom event logging... ";

    EventLogger logger;
    EventLoggerConfig config;
    config.enable_console_logging = false;

    logger.Initialize(config);

    EventLogEntry custom;
    custom.timestamp_ns = 1234567890000000ULL;
    custom.seq = 0;  // Should be auto-assigned
    custom.from_mode = ModeType::READY;
    custom.to_mode = ModeType::MANUAL_ARM;
    custom.reason = "Custom event";
    custom.is_fault = false;
    custom.fault_type = FaultType::UNKNOWN;
    custom.body_state_age_us = 100.0;
    custom.arm_state_age_us = 200.0;
    custom.arm_tracking_error = 0.05;
    custom.current_policy_id = "custom_policy";
    custom.source = "test";
    custom.detail_value = 999;

    logger.LogEvent(custom);

    auto events = logger.GetEvents();

    const EventLogEntry* found = nullptr;
    for (const auto& e : events)
    {
        if (e.reason == "Custom event")
        {
            found = &e;
            break;
        }
    }

    if (!ExpectTrue(found != nullptr, "Found custom event")) { return false; }
    if (!ExpectTrue(found->from_mode == ModeType::READY, "from_mode")) { return false; }
    if (!ExpectTrue(found->to_mode == ModeType::MANUAL_ARM, "to_mode")) { return false; }
    if (!ExpectNear(found->body_state_age_us, 100.0, "body_state_age_us")) { return false; }
    if (!ExpectStrEq(found->source, std::string("test"), "source")) { return false; }
    if (!ExpectEq(found->detail_value, 999u, "detail_value")) { return false; }

    std::cout << "PASS\n";
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::cout << "=== EventLogger Tests ===\n\n";

    bool all_passed = true;

    all_passed &= TestInitializeSuccess();
    all_passed &= TestModeToString();
    all_passed &= TestFaultToString();
    all_passed &= TestLogBasicModeTransition();
    all_passed &= TestLogModeTransitionWithContext();
    all_passed &= TestLogBasicFault();
    all_passed &= TestGetFaultEvents();
    all_passed &= TestGetEventsByMode();
    all_passed &= TestClearEvents();
    all_passed &= TestSequenceNumbers();
    all_passed &= TestCircularBuffer();
    all_passed &= TestJsonOutput();
    all_passed &= TestConcurrentLogging();
    all_passed &= TestJsonStringEscape();
    all_passed &= TestCustomEvent();

    std::cout << "\n=== " << (all_passed ? "ALL TESTS PASSED" : "SOME TESTS FAILED")
              << " ===\n";

    return all_passed ? 0 : 1;
}
