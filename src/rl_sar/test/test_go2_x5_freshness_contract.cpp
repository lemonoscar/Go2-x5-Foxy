#include <cstdlib>
#include <cstdint>
#include <iostream>

#include "rl_sar/protocol/go2_x5_protocol.hpp"
#include "rl_sar/runtime/supervisor/go2_x5_supervisor.hpp"

namespace
{

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::abort();
    }
}

void TestProtocolFreshnessHelpers()
{
    using namespace rl_sar::protocol;

    FrameHeader header;
    header.source_monotonic_ns = 1'000;

    Require(AgeNs(1'100, 1'000) == 100, "AgeNs mismatch");
    Require(IsFresh(1'100, 1'000, 100), "IsFresh should accept boundary age");
    Require(!IsFresh(1'101, 1'000, 100), "IsFresh should reject stale age");
    Require(SeqGap(10, 13) == 3, "SeqGap mismatch");
    Require(!HasSeqGap(10, 11), "HasSeqGap should accept contiguous sequence");
    Require(HasSeqGap(10, 12), "HasSeqGap should detect a gap");
    Require(!IsCommandExpired(999, 1'000), "IsCommandExpired should stay false before deadline");
    Require(IsCommandExpired(1'000, 1'000), "IsCommandExpired should trip at deadline");
    Require(IsFresh(header, 1'050, 50), "FrameHeader IsFresh overload mismatch");
}

void TestSupervisorFreshnessHelpers()
{
    using Go2X5Supervisor::Supervisor;

    Require(Supervisor::IsFresh(50, 50), "Supervisor::IsFresh should accept boundary age");
    Require(!Supervisor::IsFresh(51, 50), "Supervisor::IsFresh should reject stale age");
    Require(!Supervisor::HasSequenceGap(11, 10), "Supervisor::HasSequenceGap should accept contiguous sequence");
    Require(Supervisor::HasSequenceGap(12, 10), "Supervisor::HasSequenceGap should detect a gap");
}

void TestRuntimeFreshnessAndSeqGap()
{
    using Go2X5Supervisor::Mode;
    using Go2X5Supervisor::Supervisor;
    using Go2X5Supervisor::WatchdogInput;

    Supervisor supervisor;
    supervisor.NoteHeartbeat(1'000'000);
    Require(supervisor.HeartbeatAgeUs(1'050'000) == 50, "HeartbeatAgeUs mismatch");
    Require(supervisor.HeartbeatFresh(1'050'000), "HeartbeatFresh should accept fresh heartbeat");

    WatchdogInput seed;
    seed.now_monotonic_ns = 1'050'000;
    seed.manifest_valid = true;
    seed.body_state_age_us = 49;
    seed.arm_state_age_us = 49;
    seed.policy_age_us = 99;
    seed.arm_tracking_error_age_us = 0;
    seed.policy_health_ok = true;
    seed.body_dds_write_ok = true;
    seed.arm_backend_valid = true;
    seed.has_body_state_seq = true;
    seed.body_state_seq = 10;
    seed.has_arm_state_seq = true;
    seed.arm_state_seq = 20;
    seed.has_policy_seq = true;
    seed.policy_seq = 30;

    auto status = supervisor.EvaluateWatchdog(seed);
    Require(status.heartbeat_fresh, "watchdog heartbeat should be fresh");
    Require(status.body_state_fresh, "body state should be fresh");
    Require(status.arm_state_fresh, "arm state should be fresh");
    Require(status.policy_fresh, "policy should be fresh");
    Require(status.ready_gate_ok, "ready gate should be open");
    Require(status.active_gate_ok, "active gate should be open");
    Require(status.seq_gap_count == 0, "initial seq gap count should be zero");

    auto held = supervisor.Step(seed);
    Require(held.current_mode == Mode::Boot, "seed step should keep supervisor in BOOT");

    WatchdogInput gap = seed;
    gap.now_monotonic_ns = 1'060'000;
    gap.body_state_seq = 12;
    gap.arm_state_seq = 23;
    gap.policy_seq = 31;

    status = supervisor.EvaluateWatchdog(gap);
    Require(status.body_seq_gap, "body sequence gap should be detected");
    Require(status.arm_seq_gap, "arm sequence gap should be detected");
    Require(!status.policy_seq_gap, "policy sequence should remain contiguous");
    Require(status.seq_gap_count == 2, "seq gap count should reflect body and arm gaps");
}

void TestFreshnessThresholds()
{
    using Go2X5Supervisor::Supervisor;

    const uint64_t body_stale_us = 50;
    const uint64_t arm_stale_us = 50;
    const uint64_t policy_stale_us = 100;
    const uint64_t command_expire_ns = 2'000'000ULL;

    Require(Supervisor::IsFresh(body_stale_us, body_stale_us), "body freshness boundary should pass");
    Require(!Supervisor::IsFresh(body_stale_us + 1, body_stale_us), "body freshness over boundary should fail");
    Require(Supervisor::IsFresh(arm_stale_us, arm_stale_us), "arm freshness boundary should pass");
    Require(!Supervisor::IsFresh(arm_stale_us + 1, arm_stale_us), "arm freshness over boundary should fail");
    Require(Supervisor::IsFresh(policy_stale_us, policy_stale_us), "policy freshness boundary should pass");
    Require(!Supervisor::IsFresh(policy_stale_us + 1, policy_stale_us), "policy freshness over boundary should fail");
    Require(!rl_sar::protocol::IsCommandExpired(command_expire_ns - 1, command_expire_ns), "command should still be valid");
    Require(rl_sar::protocol::IsCommandExpired(command_expire_ns, command_expire_ns), "command should expire at boundary");
}

}  // namespace

int main()
{
    TestProtocolFreshnessHelpers();
    TestSupervisorFreshnessHelpers();
    TestRuntimeFreshnessAndSeqGap();
    TestFreshnessThresholds();
    std::cout << "test_go2_x5_freshness_contract passed\n";
    return 0;
}
