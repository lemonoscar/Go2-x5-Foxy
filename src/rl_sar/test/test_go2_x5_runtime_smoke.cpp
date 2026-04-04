#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>

#include "deploy_manifest_runtime.hpp"
#include "rl_sar/protocol/go2_x5_protocol.hpp"
#include "rl_sar/runtime/supervisor/go2_x5_supervisor.hpp"

namespace
{

using rl_sar::protocol::FrameType;
using rl_sar::protocol::HybridDiagnosticFrame;
using rl_sar::protocol::ModeEventFrame;

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

Go2X5Supervisor::Config BuildSupervisorConfig(const RLConfig::DeployManifestRuntimeSnapshot& snapshot)
{
    Go2X5Supervisor::Config config;
    config.probe_window_us = static_cast<uint64_t>(snapshot.probe_window_sec * 1'000'000.0);
    config.body_state_stale_us = static_cast<uint64_t>(snapshot.lowstate_timeout_ms) * 1000ULL;
    config.arm_state_stale_us = static_cast<uint64_t>(snapshot.arm_state_timeout_ms) * 1000ULL;
    config.policy_stale_us = static_cast<uint64_t>(snapshot.policy_timeout_ms) * 1000ULL;
    config.arm_tracking_error_window_us = 200'000ULL;
    config.degraded_timeout_us = static_cast<uint64_t>(snapshot.degraded_timeout_sec * 1'000'000.0);
    config.soft_stop_duration_us = 500'000ULL;
    config.require_manifest_valid = true;
    config.fault_latched_requires_manual_reset = true;
    return config;
}

struct FakeBodySource
{
    uint64_t seq = 1;
    uint64_t age_us = 0;
    bool dds_ok = true;
};

struct FakeArmSource
{
    uint64_t seq = 2;
    uint64_t age_us = 0;
    uint64_t tracking_error_age_us = 0;
    bool backend_valid = true;
    bool tracking_error_high = false;
};

struct FakePolicySource
{
    uint64_t seq = 3;
    uint64_t age_us = 0;
    bool health_ok = true;
    bool health_bad = false;
};

struct FakeOperatorSource
{
    bool operator_enable = false;
    bool operator_disable = false;
    bool manual_arm_request = false;
    bool estop = false;
    bool soft_stop_request = false;
    bool fault_reset = false;
};

class SmokeHarness
{
public:
    explicit SmokeHarness(const std::filesystem::path& manifest_path)
        : manifest_runtime_(manifest_path.string())
    {
        const auto load = this->manifest_runtime_.LoadFromFile(manifest_path.string());
        Require(load.is_valid, load.error_message.c_str());
        Require(this->manifest_runtime_.HasManifest(), "manifest should be loaded");

        const auto snapshot = this->manifest_runtime_.Snapshot();
        Require(snapshot.manifest_valid, "manifest snapshot should be valid");
        this->supervisor_ = Go2X5Supervisor::Supervisor(BuildSupervisorConfig(snapshot));
        this->supervisor_.NoteHeartbeat(this->now_monotonic_ns_);
    }

    void Advance(uint64_t delta_ns)
    {
        this->now_monotonic_ns_ += delta_ns;
        this->supervisor_.NoteHeartbeat(this->now_monotonic_ns_);
    }

    void SetProbePass(bool value)
    {
        this->probe_pass_ = value;
    }

    void SetOperatorEnable(bool value)
    {
        this->operator_.operator_enable = value;
        this->operator_.operator_disable = !value;
    }

    void SetOperatorDisable(bool value)
    {
        this->operator_.operator_disable = value;
    }

    void SetManualArmRequest(bool value)
    {
        this->operator_.manual_arm_request = value;
    }

    void SetEstop(bool value)
    {
        this->operator_.estop = value;
    }

    void SetFaultReset(bool value)
    {
        this->operator_.fault_reset = value;
    }

    void SetSoftStop(bool value)
    {
        this->operator_.soft_stop_request = value;
    }

    void SetBodySeq(uint64_t seq)
    {
        this->body_.seq = seq;
    }

    void SetArmSeq(uint64_t seq)
    {
        this->arm_.seq = seq;
    }

    void SetPolicySeq(uint64_t seq)
    {
        this->policy_.seq = seq;
    }

    void SetBodyFreshness(uint64_t age_us, bool dds_ok)
    {
        this->body_.age_us = age_us;
        this->body_.dds_ok = dds_ok;
    }

    void SetArmFreshness(uint64_t age_us, bool backend_valid, bool tracking_error_high, uint64_t tracking_error_age_us)
    {
        this->arm_.age_us = age_us;
        this->arm_.backend_valid = backend_valid;
        this->arm_.tracking_error_high = tracking_error_high;
        this->arm_.tracking_error_age_us = tracking_error_age_us;
    }

    void SetPolicyFreshness(uint64_t age_us, bool health_ok, bool health_bad)
    {
        this->policy_.age_us = age_us;
        this->policy_.health_ok = health_ok;
        this->policy_.health_bad = health_bad;
    }

    Go2X5Supervisor::WatchdogInput BuildInput() const
    {
        Go2X5Supervisor::WatchdogInput input;
        input.now_monotonic_ns = this->now_monotonic_ns_;
        input.manifest_valid = this->manifest_runtime_.Snapshot().manifest_valid;
        input.config_loaded = true;
        input.boot_complete = true;
        input.probe_pass = this->probe_pass_;
        input.operator_enable = this->operator_.operator_enable;
        input.operator_disable = this->operator_.operator_disable;
        input.manual_arm_request = this->operator_.manual_arm_request;
        input.estop = this->operator_.estop;
        input.soft_stop_request = this->operator_.soft_stop_request;
        input.fault_reset = this->operator_.fault_reset;
        input.policy_health_ok = this->policy_.health_ok;
        input.policy_health_bad = this->policy_.health_bad;
        input.body_dds_write_ok = this->body_.dds_ok;
        input.arm_backend_valid = this->arm_.backend_valid;
        input.arm_tracking_error_high = this->arm_.tracking_error_high;
        input.body_state_age_us = this->body_.age_us;
        input.arm_state_age_us = this->arm_.age_us;
        input.policy_age_us = this->policy_.age_us;
        input.arm_tracking_error_age_us = this->arm_.tracking_error_age_us;
        input.allow_recover =
            input.manifest_valid &&
            input.body_dds_write_ok &&
            input.arm_backend_valid &&
            input.body_state_age_us <= this->supervisor_.config().body_state_stale_us &&
            input.arm_state_age_us <= this->supervisor_.config().arm_state_stale_us &&
            input.policy_age_us <= this->supervisor_.config().policy_stale_us;
        input.has_body_state_seq = true;
        input.has_arm_state_seq = true;
        input.has_policy_seq = true;
        input.body_state_seq = this->body_.seq;
        input.arm_state_seq = this->arm_.seq;
        input.policy_seq = this->policy_.seq;
        input.probe_fail = !input.manifest_valid;
        return input;
    }

    Go2X5Supervisor::TransitionResult Step()
    {
        return this->supervisor_.Step(this->BuildInput());
    }

    Go2X5Supervisor::WatchdogStatus Status() const
    {
        return this->supervisor_.EvaluateWatchdog(this->BuildInput());
    }

    const Go2X5Supervisor::Supervisor& supervisor() const
    {
        return this->supervisor_;
    }

    const RLConfig::DeployManifestRuntime& manifest_runtime() const
    {
        return this->manifest_runtime_;
    }

    HybridDiagnosticFrame BuildDiagnosticFrame(const Go2X5Supervisor::TransitionResult& result) const
    {
        HybridDiagnosticFrame frame;
        frame.header.msg_type = FrameType::HybridDiagnostic;
        frame.header.seq = result.trigger_seq;
        frame.header.source_monotonic_ns = result.source_monotonic_ns;
        frame.body_state_age_us = static_cast<uint32_t>(result.watchdog.body_state_age_us);
        frame.arm_state_age_us = static_cast<uint32_t>(result.watchdog.arm_state_age_us);
        frame.policy_latency_us = static_cast<uint32_t>(result.watchdog.policy_age_us);
        frame.coordinator_jitter_us = static_cast<uint32_t>(result.watchdog.heartbeat_age_us);
        frame.arm_tracking_error_norm = result.watchdog.arm_tracking_error_high ? 1.0f : 0.0f;
        frame.xy_drift_error = result.watchdog.body_dds_write_ok ? 0.0f : 1.0f;
        frame.yaw_drift_error = result.watchdog.arm_backend_valid ? 0.0f : 1.0f;
        frame.seq_gap_count = result.watchdog.seq_gap_count;
        frame.dds_write_fail_count = result.watchdog.body_dds_write_ok ? 0u : 1u;
        frame.arm_backend_fail_count = result.watchdog.arm_backend_valid ? 0u : 1u;
        frame.clip_count = result.watchdog.active_gate_ok ? 0u : 1u;
        return frame;
    }

    ModeEventFrame BuildModeEventFrame(const Go2X5Supervisor::TransitionResult& result) const
    {
        ModeEventFrame frame;
        frame.header.msg_type = FrameType::ModeEvent;
        frame.header.seq = result.trigger_seq;
        frame.header.source_monotonic_ns = result.source_monotonic_ns;
        frame.from_mode = static_cast<uint16_t>(result.previous_mode);
        frame.to_mode = static_cast<uint16_t>(result.current_mode);
        frame.reason_code = static_cast<uint32_t>(result.reason_code);
        frame.trigger_seq = result.trigger_seq;
        frame.detail_value = result.detail_value;
        return frame;
    }

private:
    RLConfig::DeployManifestRuntime manifest_runtime_;
    Go2X5Supervisor::Supervisor supervisor_;
    FakeBodySource body_;
    FakeArmSource arm_;
    FakePolicySource policy_;
    FakeOperatorSource operator_;
    bool probe_pass_ = false;
    uint64_t now_monotonic_ns_ = 1'000'000;
};

void RequireModeEventRoundTrip(const SmokeHarness& harness, const Go2X5Supervisor::TransitionResult& result)
{
    const auto frame = harness.BuildModeEventFrame(result);
    const auto bytes = rl_sar::protocol::SerializeModeEventFrame(frame);
    rl_sar::protocol::ModeEventFrame parsed;
    std::string error;
    Require(rl_sar::protocol::ParseModeEventFrame(bytes, parsed, &error), error.c_str());
    Require(parsed.from_mode == frame.from_mode, "mode event from_mode mismatch");
    Require(parsed.to_mode == frame.to_mode, "mode event to_mode mismatch");
    Require(parsed.reason_code == frame.reason_code, "mode event reason mismatch");
    Require(parsed.trigger_seq == frame.trigger_seq, "mode event trigger seq mismatch");
    Require(parsed.detail_value == frame.detail_value, "mode event detail mismatch");
}

void RequireDiagnosticRoundTrip(const SmokeHarness& harness, const Go2X5Supervisor::TransitionResult& result)
{
    const auto frame = harness.BuildDiagnosticFrame(result);
    const auto bytes = rl_sar::protocol::SerializeHybridDiagnosticFrame(frame);
    rl_sar::protocol::HybridDiagnosticFrame parsed;
    std::string error;
    Require(rl_sar::protocol::ParseHybridDiagnosticFrame(bytes, parsed, &error), error.c_str());
    Require(parsed.body_state_age_us == frame.body_state_age_us, "diagnostic body age mismatch");
    Require(parsed.arm_state_age_us == frame.arm_state_age_us, "diagnostic arm age mismatch");
    Require(parsed.policy_latency_us == frame.policy_latency_us, "diagnostic policy latency mismatch");
    Require(parsed.seq_gap_count == frame.seq_gap_count, "diagnostic seq gap mismatch");
}

void TestManifestLoadAndModeTransitions()
{
    const auto repo_root = std::filesystem::path(RL_SAR_SOURCE_DIR).parent_path().parent_path();
    const auto manifest_path = repo_root / RLConfig::DeployManifestLoader::kDefaultManifestPath;

    Require(std::filesystem::exists(manifest_path), "manifest file missing");

    SmokeHarness harness(manifest_path);
    const auto snapshot = harness.manifest_runtime().Snapshot();
    Require(snapshot.manifest_valid, "manifest snapshot should be valid");
    Require(snapshot.manifest_hash == harness.manifest_runtime().ManifestHash(), "manifest hash mismatch");
    Require(snapshot.protocol_version == 1, "protocol version mismatch");
    Require(snapshot.arm_bridge_transport == "ipc", "runtime smoke should use ipc mirror transport");
    Require(snapshot.arm_cmd_topic == "/arx_x5/joint_cmd", "runtime smoke arm cmd topic mismatch");
    Require(snapshot.arm_state_topic == "/arx_x5/joint_state", "runtime smoke arm state topic mismatch");
    Require(snapshot.arm_joint_command_topic == "/arm_joint_pos_cmd",
        "runtime smoke arm joint command topic mismatch");
    Require(snapshot.bridge_rmw_implementation == "rmw_cyclonedds_cpp", "runtime smoke bridge RMW mismatch");
    Require(snapshot.go2_rmw_implementation == "rmw_fastrtps_cpp", "runtime smoke runtime RMW mismatch");

    auto boot = harness.Step();
    Require(boot.current_mode == Go2X5Supervisor::Mode::Probe, "BOOT -> PROBE transition failed in smoke");
    Require(!harness.supervisor().event_log().empty(), "boot transition should be logged");
    Require(harness.supervisor().event_log().back().source_label == "supervisor",
        "boot transition should record supervisor source");
    RequireModeEventRoundTrip(harness, boot);
    RequireDiagnosticRoundTrip(harness, boot);

    harness.SetProbePass(true);
    harness.Advance(20'000);
    auto probe = harness.Step();
    Require(probe.current_mode == Go2X5Supervisor::Mode::Passive, "PROBE -> PASSIVE transition failed in smoke");
    RequireModeEventRoundTrip(harness, probe);
    RequireDiagnosticRoundTrip(harness, probe);

    harness.Advance(20'000);
    auto ready = harness.Step();
    Require(ready.current_mode == Go2X5Supervisor::Mode::Ready, "PASSIVE -> READY transition failed in smoke");
    RequireModeEventRoundTrip(harness, ready);
    RequireDiagnosticRoundTrip(harness, ready);

    harness.SetOperatorEnable(true);
    harness.Advance(20'000);
    auto active = harness.Step();
    Require(active.current_mode == Go2X5Supervisor::Mode::RlDogOnlyActive,
        "READY -> RL_DOG_ONLY_ACTIVE transition failed in smoke");
    RequireModeEventRoundTrip(harness, active);
    RequireDiagnosticRoundTrip(harness, active);

    harness.SetBodySeq(2);
    harness.SetArmSeq(3);
    harness.SetPolicySeq(4);
    auto seq_status = harness.Status();
    Require(seq_status.seq_gap_count == 0, "initial smoke seq gap count should be zero");

    harness.SetBodySeq(3);
    harness.SetArmSeq(4);
    harness.SetPolicySeq(4);
    seq_status = harness.Status();
    Require(seq_status.body_seq_gap, "smoke should observe body seq gap");
    Require(seq_status.arm_seq_gap, "smoke should observe arm seq gap");
    Require(!seq_status.policy_seq_gap, "smoke should keep policy seq contiguous");
    Require(seq_status.seq_gap_count == 2, "smoke seq gap count mismatch");

    harness.SetBodyFreshness(harness.supervisor().config().body_state_stale_us + 1, true);
    harness.SetArmFreshness(0, true, false, 0);
    harness.SetPolicyFreshness(0, true, false);
    harness.Advance(20'000);
    auto degraded_body = harness.Step();
    Require(degraded_body.current_mode == Go2X5Supervisor::Mode::DegradedBody,
        "body freshness should drive DEGRADED_BODY in smoke");
    RequireModeEventRoundTrip(harness, degraded_body);
    RequireDiagnosticRoundTrip(harness, degraded_body);

    harness.SetBodyFreshness(0, true);
    harness.SetArmFreshness(0, true, false, 0);
    harness.SetPolicyFreshness(0, true, false);
    harness.SetOperatorEnable(false);
    harness.SetOperatorDisable(false);
    harness.Advance(20'000);
    auto recovered = harness.Step();
    Require(recovered.current_mode == Go2X5Supervisor::Mode::Ready,
        "degraded body recovery should return to READY in smoke");
    RequireModeEventRoundTrip(harness, recovered);
    RequireDiagnosticRoundTrip(harness, recovered);

    harness.SetManualArmRequest(true);
    harness.Advance(20'000);
    auto manual_arm = harness.Step();
    Require(manual_arm.current_mode == Go2X5Supervisor::Mode::ManualArm,
        "manual arm request should enter MANUAL_ARM in smoke");
    Require(harness.supervisor().event_log().back().source_label == "operator",
        "manual arm transition should record operator source");
    RequireModeEventRoundTrip(harness, manual_arm);
    RequireDiagnosticRoundTrip(harness, manual_arm);

    harness.SetEstop(true);
    harness.Advance(20'000);
    auto fault = harness.Step();
    Require(fault.current_mode == Go2X5Supervisor::Mode::FaultLatched,
        "estop should latch fault in smoke");
    Require(harness.supervisor().event_log().back().source_label == "operator",
        "estop transition should record operator source");
    RequireModeEventRoundTrip(harness, fault);
    RequireDiagnosticRoundTrip(harness, fault);

    harness.SetEstop(false);
    harness.SetManualArmRequest(false);
    harness.SetFaultReset(true);
    harness.SetProbePass(true);
    harness.Advance(20'000);
    auto reset = harness.Step();
    Require(reset.current_mode == Go2X5Supervisor::Mode::Passive,
        "fault reset should recover to PASSIVE in smoke");
    RequireModeEventRoundTrip(harness, reset);
    RequireDiagnosticRoundTrip(harness, reset);
}

} // namespace

int main()
{
    TestManifestLoadAndModeTransitions();
    std::cout << "test_go2_x5_runtime_smoke passed\n";
    return 0;
}
