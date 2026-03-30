#include <chrono>
#include <iostream>
#include <limits>
#include <vector>

#include "rl_sar/go2x5/safety/go2_x5_safety_guard.hpp"

int main()
{
    using namespace Go2X5SafetyGuard;

    ArmSafetyContext safety_context;
    safety_context.arm_joint_start_index = 0;
    safety_context.arm_joint_count = 2;
    safety_context.arm_joint_lower_limits = {-1.0f, -2.0f};
    safety_context.arm_joint_upper_limits = {1.0f, 2.0f};
    safety_context.whole_body_velocity_limits = {0.5f, 0.75f};
    safety_context.whole_body_effort_limits = {10.0f, 20.0f};
    safety_context.whole_body_kp_limits = {100.0f, 50.0f};
    safety_context.whole_body_kd_limits = {5.0f, 6.0f};

    {
        std::vector<float> target = {std::numeric_limits<float>::quiet_NaN(), 3.0f};
        const std::vector<float> fallback = {0.25f, -0.5f};
        if (!ClipArmPoseTargetInPlace(&target, fallback, safety_context, "pose"))
        {
            std::cerr << "Expected pose clip to succeed\n";
            return 1;
        }
        if (target != std::vector<float>({0.25f, 2.0f}))
        {
            std::cerr << "Expected pose clip to sanitize non-finite values and clamp upper limits\n";
            return 1;
        }
    }

    {
        const std::vector<float> valid = {0.0f, 1.5f};
        if (!ValidateArmPoseTarget(valid, safety_context, "pose"))
        {
            std::cerr << "Expected valid arm pose target to pass\n";
            return 1;
        }
        const std::vector<float> invalid = {0.0f, 2.5f};
        if (ValidateArmPoseTarget(invalid, safety_context, "pose"))
        {
            std::cerr << "Expected out-of-range arm pose target to fail\n";
            return 1;
        }
    }

    {
        std::vector<float> q = {2.0f, -3.0f};
        std::vector<float> dq = {1.0f, -2.0f};
        std::vector<float> kp = {200.0f, -1.0f};
        std::vector<float> kd = {10.0f, std::numeric_limits<float>::quiet_NaN()};
        std::vector<float> tau = {11.0f, -21.0f};
        if (!ClipArmBridgeCommandInPlace(&q, &dq, &kp, &kd, &tau, {0.0f, 0.0f}, safety_context, "bridge"))
        {
            std::cerr << "Expected arm bridge command clip to succeed\n";
            return 1;
        }
        if (q != std::vector<float>({1.0f, -2.0f}) ||
            dq != std::vector<float>({0.5f, -0.75f}) ||
            kp != std::vector<float>({100.0f, 0.0f}) ||
            kd != std::vector<float>({5.0f, 0.0f}) ||
            tau != std::vector<float>({10.0f, -20.0f}))
        {
            std::cerr << "Expected bridge command clip to enforce deploy limits\n";
            return 1;
        }
    }

    {
        const std::vector<float> q = {0.5f, 1.0f};
        const std::vector<float> dq = {0.1f, 0.2f};
        const std::vector<float> tau = {0.0f, 0.0f};
        if (!ValidateArmBridgeStateSample(q, dq, tau, safety_context, "state"))
        {
            std::cerr << "Expected plausible bridge state sample to pass\n";
            return 1;
        }
        const std::vector<float> stale_q = {1.6f, 0.0f};
        if (ValidateArmBridgeStateSample(stale_q, dq, tau, safety_context, "state"))
        {
            std::cerr << "Expected implausible bridge state sample to fail\n";
            return 1;
        }
    }

    {
        const auto now = std::chrono::steady_clock::now();
        ArmBridgeStateSnapshot state_snapshot;
        state_snapshot.state_valid = true;
        state_snapshot.require_live_state = true;
        state_snapshot.state_from_backend = true;
        state_snapshot.state_timeout_sec = 0.25f;
        state_snapshot.state_stamp = now - std::chrono::milliseconds(100);
        if (!IsArmBridgeStateFresh(state_snapshot, now))
        {
            std::cerr << "Expected fresh bridge state to pass\n";
            return 1;
        }

        state_snapshot.state_from_backend = false;
        if (IsArmBridgeStateFresh(state_snapshot, now))
        {
            std::cerr << "Expected shadow-only bridge state to fail when live state is required\n";
            return 1;
        }

        state_snapshot.state_from_backend = true;
        state_snapshot.state_stamp = now - std::chrono::milliseconds(500);
        if (IsArmBridgeStateFresh(state_snapshot, now))
        {
            std::cerr << "Expected stale bridge state to fail\n";
            return 1;
        }
    }

    std::cout << "go2_x5 safety guard test passed." << std::endl;
    return 0;
}
