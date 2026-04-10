#include <cstdlib>
#include <cstdint>
#include <iostream>

#include "rl_sar/runtime/supervisor/go2_x5_supervisor.hpp"

namespace
{

using Go2X5Supervisor::Mode;
using Go2X5Supervisor::EventType;
using Go2X5Supervisor::ReasonCode;
using Go2X5Supervisor::Supervisor;
using Go2X5Supervisor::WatchdogInput;

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

WatchdogInput MakeHealthyInput(std::uint64_t now_monotonic_ns, std::uint64_t seq)
{
    WatchdogInput input;
    input.now_monotonic_ns = now_monotonic_ns;
    input.manifest_valid = true;
    input.config_loaded = false;
    input.boot_complete = false;
    input.probe_pass = false;
    input.policy_health_ok = true;
    input.body_dds_write_ok = true;
    input.arm_backend_valid = true;
    input.body_state_age_us = 0;
    input.arm_state_age_us = 0;
    input.policy_age_us = 0;
    input.arm_tracking_error_age_us = 0;
    input.has_body_state_seq = true;
    input.has_arm_state_seq = true;
    input.has_policy_seq = true;
    input.body_state_seq = seq;
    input.arm_state_seq = seq + 1;
    input.policy_seq = seq + 2;
    return input;
}

WatchdogInput MakeBootInput(std::uint64_t now_monotonic_ns, std::uint64_t seq)
{
    auto input = MakeHealthyInput(now_monotonic_ns, seq);
    input.config_loaded = true;
    input.boot_complete = true;
    return input;
}

WatchdogInput MakeProbePassInput(std::uint64_t now_monotonic_ns, std::uint64_t seq)
{
    auto input = MakeHealthyInput(now_monotonic_ns, seq);
    input.probe_pass = true;
    return input;
}

void DriveToReady(Supervisor& supervisor, std::uint64_t now_monotonic_ns, std::uint64_t seq)
{
    supervisor.Reset();
    supervisor.NoteHeartbeat(now_monotonic_ns);

    auto boot = supervisor.Step(MakeBootInput(now_monotonic_ns, seq));
    Require(boot.current_mode == Mode::Probe, "BOOT -> PROBE transition failed");

    auto probe = supervisor.Step(MakeProbePassInput(now_monotonic_ns + 20'000, seq + 1));
    Require(probe.current_mode == Mode::Passive, "PROBE -> PASSIVE transition failed");

    auto ready = supervisor.Step(MakeHealthyInput(now_monotonic_ns + 40'000, seq + 2));
    Require(ready.current_mode == Mode::Ready, "PASSIVE -> READY transition failed");
}

void DriveToActive(Supervisor& supervisor, std::uint64_t now_monotonic_ns, std::uint64_t seq)
{
    DriveToReady(supervisor, now_monotonic_ns, seq);

    auto active = MakeHealthyInput(now_monotonic_ns + 60'000, seq + 3);
    active.operator_enable = true;
    auto active_result = supervisor.Step(active);
    Require(active_result.current_mode == Mode::RlDogOnlyActive, "READY -> RL_DOG_ONLY_ACTIVE transition failed");
}

void RequireLastEvent(const Supervisor& supervisor,
                      EventType event_type,
                      Mode from_mode,
                      Mode to_mode,
                      ReasonCode reason_code,
                      const char* source_label)
{
    const auto& events = supervisor.event_log();
    Require(!events.empty(), "supervisor event log should not be empty");
    const auto& event = events.back();
    Require(event.event_type == event_type, "unexpected supervisor event type");
    Require(event.from_mode == from_mode, "unexpected supervisor event source mode");
    Require(event.to_mode == to_mode, "unexpected supervisor event target mode");
    Require(event.reason_code == reason_code, "unexpected supervisor event reason");
    Require(event.source_label == source_label, "unexpected supervisor event source");
}

void TestRuntimeTransitions()
{
    Supervisor supervisor;
    supervisor.NoteHeartbeat(1'000'000);

    WatchdogInput boot = MakeHealthyInput(1'000'000, 1);
    boot.config_loaded = true;
    boot.boot_complete = true;
    auto boot_result = supervisor.Step(boot);
    Require(boot_result.current_mode == Mode::Probe, "BOOT -> PROBE transition failed");
    Require(supervisor.mode() == Mode::Probe, "supervisor mode should be PROBE");

    WatchdogInput probe = MakeHealthyInput(1'020'000, 2);
    probe.probe_pass = true;
    auto probe_result = supervisor.Step(probe);
    Require(probe_result.current_mode == Mode::Passive, "PROBE -> PASSIVE transition failed");

    WatchdogInput ready = MakeHealthyInput(1'040'000, 3);
    auto ready_result = supervisor.Step(ready);
    Require(ready_result.current_mode == Mode::Ready, "PASSIVE -> READY transition failed");

    WatchdogInput active = MakeHealthyInput(1'060'000, 4);
    active.operator_enable = true;
    auto active_result = supervisor.Step(active);
    Require(active_result.current_mode == Mode::RlDogOnlyActive, "READY -> RL_DOG_ONLY_ACTIVE transition failed");

    WatchdogInput arm_stale = MakeHealthyInput(1'080'000, 5);
    arm_stale.operator_enable = true;
    arm_stale.arm_state_age_us = 60'000;
    auto degraded_arm = supervisor.Step(arm_stale);
    Require(degraded_arm.current_mode == Mode::DegradedArm, "active -> DEGRADED_ARM transition failed");

    WatchdogInput recover_arm = MakeHealthyInput(1'100'000, 6);
    recover_arm.allow_recover = true;
    recover_arm.arm_state_age_us = 20'000;
    recover_arm.operator_enable = true;
    auto recovered_arm = supervisor.Step(recover_arm);
    Require(recovered_arm.current_mode == Mode::Ready, "DEGRADED_ARM -> READY recovery failed");

    WatchdogInput soft_stop = MakeHealthyInput(1'120'000, 7);
    soft_stop.soft_stop_request = true;
    auto soft_stop_result = supervisor.Step(soft_stop);
    Require(soft_stop_result.current_mode == Mode::SoftStop, "READY -> SOFT_STOP transition failed");

    WatchdogInput soft_timeout = MakeHealthyInput(1'700'000'000, 8);
    auto soft_timeout_result = supervisor.Step(soft_timeout);
    Require(soft_timeout_result.current_mode == Mode::Passive, "SOFT_STOP timeout -> PASSIVE transition failed");
}

void TestManifestValidityPath()
{
    Supervisor supervisor;
    supervisor.NoteHeartbeat(2'000'000);

    WatchdogInput invalid_manifest = MakeHealthyInput(2'000'000, 1);
    invalid_manifest.config_loaded = true;
    invalid_manifest.boot_complete = true;
    invalid_manifest.manifest_valid = false;
    auto faulted = supervisor.Step(invalid_manifest);
    Require(faulted.current_mode == Mode::FaultLatched, "invalid manifest should latch fault");
    Require(supervisor.IsLatchedFault(), "supervisor should report latched fault");
    RequireLastEvent(supervisor, EventType::ProbeFail, Mode::Boot, Mode::FaultLatched,
        ReasonCode::ManifestMismatch, "manifest");

    WatchdogInput reset = MakeHealthyInput(2'020'000, 2);
    reset.manifest_valid = true;
    reset.fault_reset = true;
    reset.probe_pass = true;
    auto reset_result = supervisor.Step(reset);
    Require(reset_result.current_mode == Mode::Passive, "FAULT_LATCHED reset should recover to PASSIVE");
}

void TestSeqGapAndWatchdogStatus()
{
    Supervisor supervisor;
    supervisor.NoteHeartbeat(3'000'000);

    WatchdogInput seed = MakeHealthyInput(3'000'000, 10);
    auto seed_result = supervisor.Step(seed);
    Require(seed_result.current_mode == Mode::Boot, "seed step should stay in BOOT");

    WatchdogInput gap = MakeHealthyInput(3'020'000, 12);
    gap.policy_seq = 13;
    auto status = supervisor.EvaluateWatchdog(gap);
    Require(status.body_seq_gap, "body sequence gap should be reported");
    Require(status.arm_seq_gap, "arm sequence gap should be reported");
    Require(!status.policy_seq_gap, "policy sequence should remain contiguous");
    Require(status.seq_gap_count == 2, "seq gap count should be 2");
    Require(status.ready_gate_ok, "watchdog should still allow READY on fresh data");
    Require(status.active_gate_ok, "watchdog should still allow active mode on fresh data");
}

void TestEstopAndFaultResetInputs()
{
    Supervisor supervisor;
    DriveToActive(supervisor, 4'000'000, 40);
    supervisor.ClearEventLog();

    WatchdogInput estop = MakeHealthyInput(4'060'000, 50);
    estop.operator_enable = true;
    estop.estop = true;
    auto estop_result = supervisor.Step(estop);
    Require(estop_result.current_mode == Mode::FaultLatched, "estop should latch fault");
    RequireLastEvent(supervisor, EventType::Estop, Mode::RlDogOnlyActive, Mode::FaultLatched, ReasonCode::Estop,
        "operator");

    WatchdogInput blocked_reset = MakeHealthyInput(4'080'000, 51);
    blocked_reset.fault_reset = true;
    auto blocked_reset_result = supervisor.Step(blocked_reset);
    Require(blocked_reset_result.current_mode == Mode::FaultLatched,
        "fault_reset without probe_pass should stay latched");

    WatchdogInput good_reset = MakeHealthyInput(4'100'000, 52);
    good_reset.fault_reset = true;
    good_reset.probe_pass = true;
    auto good_reset_result = supervisor.Step(good_reset);
    Require(good_reset_result.current_mode == Mode::Passive, "fault_reset + probe_pass should recover to PASSIVE");
    RequireLastEvent(supervisor, EventType::FaultReset, Mode::FaultLatched, Mode::Passive, ReasonCode::None,
        "operator");
}

void TestEstopIsGlobalAndLatched()
{
    {
        Supervisor supervisor;
        supervisor.NoteHeartbeat(4'200'000);
        WatchdogInput boot_estop = MakeBootInput(4'200'000, 60);
        boot_estop.estop = true;
        auto result = supervisor.Step(boot_estop);
        Require(result.current_mode == Mode::FaultLatched, "BOOT estop should latch fault");
        RequireLastEvent(supervisor, EventType::Estop, Mode::Boot, Mode::FaultLatched, ReasonCode::Estop,
            "operator");
    }

    {
        Supervisor supervisor;
        supervisor.NoteHeartbeat(4'300'000);
        auto boot = supervisor.Step(MakeBootInput(4'300'000, 70));
        Require(boot.current_mode == Mode::Probe, "BOOT -> PROBE setup failed");
        supervisor.ClearEventLog();

        WatchdogInput probe_estop = MakeHealthyInput(4'320'000, 71);
        probe_estop.estop = true;
        auto result = supervisor.Step(probe_estop);
        Require(result.current_mode == Mode::FaultLatched, "PROBE estop should latch fault");
        RequireLastEvent(supervisor, EventType::Estop, Mode::Probe, Mode::FaultLatched, ReasonCode::Estop,
            "operator");
    }

    {
        Supervisor supervisor;
        supervisor.NoteHeartbeat(4'400'000);
        auto boot = supervisor.Step(MakeBootInput(4'400'000, 80));
        Require(boot.current_mode == Mode::Probe, "BOOT -> PROBE setup failed");
        auto probe = supervisor.Step(MakeProbePassInput(4'420'000, 81));
        Require(probe.current_mode == Mode::Passive, "PROBE -> PASSIVE setup failed");
        supervisor.ClearEventLog();

        WatchdogInput passive_estop = MakeHealthyInput(4'440'000, 82);
        passive_estop.estop = true;
        auto result = supervisor.Step(passive_estop);
        Require(result.current_mode == Mode::FaultLatched, "PASSIVE estop should latch fault");
        RequireLastEvent(supervisor, EventType::Estop, Mode::Passive, Mode::FaultLatched, ReasonCode::Estop,
            "operator");

        WatchdogInput reset_while_estop = MakeProbePassInput(4'460'000, 83);
        reset_while_estop.estop = true;
        reset_while_estop.fault_reset = true;
        auto blocked = supervisor.Step(reset_while_estop);
        Require(blocked.current_mode == Mode::FaultLatched,
            "fault reset must not clear a still-asserted estop");
    }
}

void TestManualArmRequestInput()
{
    Supervisor supervisor;
    DriveToReady(supervisor, 5'000'000, 60);
    supervisor.ClearEventLog();

    WatchdogInput manual_arm = MakeHealthyInput(5'060'000, 70);
    manual_arm.manual_arm_request = true;
    auto manual_arm_result = supervisor.Step(manual_arm);
    Require(manual_arm_result.current_mode == Mode::ManualArm, "manual_arm_request should enter MANUAL_ARM");
    RequireLastEvent(supervisor, EventType::OperatorEnable, Mode::Ready, Mode::ManualArm, ReasonCode::None,
        "operator");

    Supervisor active_supervisor;
    DriveToActive(active_supervisor, 5'200'000, 80);
    active_supervisor.ClearEventLog();

    WatchdogInput ignored_manual_arm = MakeHealthyInput(5'260'000, 90);
    ignored_manual_arm.operator_enable = true;
    ignored_manual_arm.manual_arm_request = true;
    auto ignored_result = active_supervisor.Step(ignored_manual_arm);
    Require(ignored_result.current_mode == Mode::RlDogOnlyActive,
        "manual_arm_request must not bypass RL_DOG_ONLY_ACTIVE");
    Require(active_supervisor.event_log().empty(), "ignored manual arm request should not emit a transition");
}

void TestRuntimeHealthInputPaths()
{
    Supervisor policy_supervisor;
    DriveToActive(policy_supervisor, 6'000'000, 100);
    policy_supervisor.ClearEventLog();

    WatchdogInput policy_ok = MakeHealthyInput(6'040'000, 109);
    policy_ok.operator_enable = true;
    auto policy_ok_result = policy_supervisor.Step(policy_ok);
    Require(policy_ok_result.current_mode == Mode::RlDogOnlyActive,
        "healthy policy input should keep RL_DOG_ONLY_ACTIVE");
    Require(policy_supervisor.event_log().empty(), "healthy policy input should not emit a transition");

    WatchdogInput policy_bad = MakeHealthyInput(6'060'000, 110);
    policy_bad.operator_enable = true;
    policy_bad.policy_health_bad = true;
    auto policy_bad_result = policy_supervisor.Step(policy_bad);
    Require(policy_bad_result.current_mode == Mode::SoftStop, "policy_health_bad should force SOFT_STOP");
    RequireLastEvent(policy_supervisor, EventType::PolicyHealthBad, Mode::RlDogOnlyActive, Mode::SoftStop,
        ReasonCode::PolicyTimeout, "policy");

    Supervisor body_supervisor;
    DriveToActive(body_supervisor, 6'200'000, 120);
    body_supervisor.ClearEventLog();

    WatchdogInput body_ok = MakeHealthyInput(6'240'000, 129);
    body_ok.operator_enable = true;
    body_ok.body_state_age_us = 10'000;
    auto body_ok_result = body_supervisor.Step(body_ok);
    Require(body_ok_result.current_mode == Mode::RlDogOnlyActive,
        "healthy body input should keep RL_DOG_ONLY_ACTIVE");
    Require(body_supervisor.event_log().empty(), "healthy body input should not emit a transition");

    WatchdogInput body_bad = MakeHealthyInput(6'260'000, 130);
    body_bad.operator_enable = true;
    body_bad.body_state_age_us = 10'000;
    body_bad.body_dds_write_ok = false;
    auto body_bad_result = body_supervisor.Step(body_bad);
    Require(body_bad_result.current_mode == Mode::DegradedBody, "body_dds_write_ok=false should degrade body");
    RequireLastEvent(body_supervisor, EventType::BodyStateStale, Mode::RlDogOnlyActive, Mode::DegradedBody,
        ReasonCode::DdsWriteFail, "body");

    Supervisor arm_supervisor;
    DriveToActive(arm_supervisor, 6'400'000, 140);
    arm_supervisor.ClearEventLog();

    WatchdogInput arm_ok = MakeHealthyInput(6'440'000, 149);
    arm_ok.operator_enable = true;
    arm_ok.arm_tracking_error_age_us = 10'000;
    auto arm_ok_result = arm_supervisor.Step(arm_ok);
    Require(arm_ok_result.current_mode == Mode::RlDogOnlyActive,
        "healthy arm input should keep RL_DOG_ONLY_ACTIVE");
    Require(arm_supervisor.event_log().empty(), "healthy arm input should not emit a transition");

    WatchdogInput arm_bad = MakeHealthyInput(6'460'000, 150);
    arm_bad.operator_enable = true;
    arm_bad.arm_tracking_error_age_us = 10'000;
    arm_bad.arm_tracking_error_high = true;
    auto arm_bad_result = arm_supervisor.Step(arm_bad);
    Require(arm_bad_result.current_mode == Mode::DegradedArm, "arm_tracking_error_high should degrade arm");
    RequireLastEvent(arm_supervisor, EventType::ArmTrackingErrorHigh, Mode::RlDogOnlyActive, Mode::DegradedArm,
        ReasonCode::ArmTrackingErrorTooHigh, "arm");
}

} // namespace

int main()
{
    TestRuntimeTransitions();
    TestManifestValidityPath();
    TestSeqGapAndWatchdogStatus();
    TestEstopAndFaultResetInputs();
    TestEstopIsGlobalAndLatched();
    TestManualArmRequestInput();
    TestRuntimeHealthInputPaths();
    std::cout << "test_go2_x5_supervisor_contract passed\n";
    return 0;
}
