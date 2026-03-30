#include <chrono>
#include <iostream>
#include <vector>

#include "rl_sar/go2x5/arm/go2_x5_arm_bridge_runtime.hpp"

namespace
{

bool RequireEqual(const std::vector<float>& actual,
                  const std::vector<float>& expected,
                  const char* label)
{
    if (actual != expected)
    {
        std::cerr << label << " mismatch\n";
        return false;
    }
    return true;
}

} // namespace

int main()
{
    using namespace Go2X5ArmBridgeRuntime;

    {
        RuntimeState state;
        state.joint_count = 2;
        state.state_valid = true;
        state.state_from_backend = true;
        state.state_timeout_warned = true;
        state.shadow_mode_warned = true;
        state.state_stream_logged = true;
        state.external_state_q = {1.0f, 2.0f};
        state.external_state_dq = {0.1f, 0.2f};
        state.external_state_tau = {0.3f, 0.4f};
        state.shadow_q = {3.0f, 4.0f};
        state.shadow_dq = {0.5f, 0.6f};

        InitializationConfig config;
        config.joint_count = 2;
        config.command_size = 2;
        config.require_state = true;
        config.require_live_state = true;
        config.state_timeout_sec = -1.0f;
        config.hold_position = {9.0f, 9.0f};
        ReconcileConfiguration(&state, config);

        if (state.state_timeout_sec != 0.0f || state.state_timeout_warned || state.shadow_mode_warned)
        {
            std::cerr << "Expected reconcile to clamp timeout and reset warnings\n";
            return 1;
        }
        if (!state.state_valid || !state.state_from_backend)
        {
            std::cerr << "Expected reconcile to preserve live state when joint count is unchanged\n";
            return 1;
        }
        if (!RequireEqual(state.external_state_q, {1.0f, 2.0f}, "preserved live q") ||
            !RequireEqual(state.shadow_q, {3.0f, 4.0f}, "preserved shadow q"))
        {
            return 1;
        }
    }

    {
        RuntimeState state;
        state.joint_count = 2;
        state.state_valid = true;
        state.state_from_backend = true;
        state.external_state_q = {1.0f, 2.0f};
        state.shadow_q = {3.0f, 4.0f};

        InitializationConfig config;
        config.joint_count = 3;
        config.command_size = 3;
        config.hold_position = {0.5f, -0.5f, 0.25f};
        ReconcileConfiguration(&state, config);

        if (state.state_valid || state.state_from_backend)
        {
            std::cerr << "Expected reconcile to invalidate bridge state when joint count changes\n";
            return 1;
        }
        if (!RequireEqual(state.shadow_q, {0.5f, -0.5f, 0.25f}, "seeded shadow q"))
        {
            return 1;
        }
    }

    {
        StateSample sample;
        if (!ParseStatePayload({1, 2, 3, 4, 5, 6}, 2, &sample) ||
            !RequireEqual(sample.q, {1, 2}, "parsed q") ||
            !RequireEqual(sample.dq, {3, 4}, "parsed dq") ||
            !RequireEqual(sample.tau, {5, 6}, "parsed tau"))
        {
            return 1;
        }
        if (!ParseStatePayload({7, 8}, 2, &sample) ||
            !RequireEqual(sample.q, {7, 8}, "partial parsed q") ||
            !RequireEqual(sample.dq, {0, 0}, "partial parsed dq"))
        {
            return 1;
        }
    }

    {
        RuntimeState state;
        InitializationConfig config;
        config.joint_count = 2;
        config.command_size = 2;
        config.require_state = true;
        config.require_live_state = true;
        config.hold_position = {0.0f, 0.0f};
        ReconcileConfiguration(&state, config);

        const auto now = std::chrono::steady_clock::now();
        StateSample sample;
        sample.q = {1.0f, 2.0f};
        sample.dq = {0.1f, 0.2f};
        sample.tau = {0.0f, 0.0f};
        auto result = ApplyStateSample(&state, sample, false, now);
        if (!result.accepted || !result.warn_shadow_only || state.state_from_backend)
        {
            std::cerr << "Expected shadow-only sample to be accepted and warned once\n";
            return 1;
        }
        result = ApplyStateSample(&state, sample, false, now);
        if (result.warn_shadow_only)
        {
            std::cerr << "Expected repeated shadow-only sample not to retrigger the warning immediately\n";
            return 1;
        }

        const auto decision = EvaluateReadDecision(state, now);
        if (decision.mode != FeedbackMode::ShadowState || !decision.shadow_only_state)
        {
            std::cerr << "Expected read decision to fall back to shadow state for shadow-only feedback\n";
            return 1;
        }
    }

    {
        RuntimeState state;
        InitializationConfig config;
        config.joint_count = 2;
        config.command_size = 2;
        config.require_state = true;
        config.require_live_state = true;
        config.hold_position = {0.0f, 0.0f};
        ReconcileConfiguration(&state, config);

        StateSample sample;
        sample.q = {1.0f, 2.0f};
        sample.dq = {0.1f, 0.2f};
        sample.tau = {0.0f, 0.0f};
        const auto now = std::chrono::steady_clock::now();
        auto result = ApplyStateSample(&state, sample, true, now);
        if (!result.accepted || !result.stream_detected || !state.state_from_backend)
        {
            std::cerr << "Expected first backend sample to mark stream detection\n";
            return 1;
        }
        result = ApplyStateSample(&state, sample, true, now);
        if (result.stream_detected)
        {
            std::cerr << "Expected stream detection to log only once\n";
            return 1;
        }

        auto decision = EvaluateReadDecision(state, now);
        if (decision.mode != FeedbackMode::LiveState || !decision.bridge_state_fresh)
        {
            std::cerr << "Expected live backend feedback to be selected\n";
            return 1;
        }

        state.state_timeout_warned = true;
        decision = EvaluateReadDecision(state, now);
        ApplyReadDecision(&state, decision);
        if (state.state_timeout_warned || state.shadow_mode_warned)
        {
            std::cerr << "Expected fresh live state to clear warning latches\n";
            return 1;
        }

        UpdateShadowState(&state, {0.5f, -0.5f}, {0.0f, 0.0f});
        if (!RequireEqual(state.shadow_q, {0.5f, -0.5f}, "updated shadow q"))
        {
            return 1;
        }
    }

    {
        RuntimeState state;
        InitializationConfig config;
        config.joint_count = 2;
        config.command_size = 2;
        config.require_state = true;
        config.require_live_state = false;
        config.state_timeout_sec = 0.1f;
        config.hold_position = {0.0f, 0.0f};
        ReconcileConfiguration(&state, config);
        state.state_valid = true;
        state.state_from_backend = true;
        state.state_stamp = std::chrono::steady_clock::now() - std::chrono::milliseconds(500);
        state.shadow_q = {0.0f, 0.0f};
        state.shadow_dq = {0.0f, 0.0f};

        const auto decision = EvaluateReadDecision(state, std::chrono::steady_clock::now());
        if (decision.mode != FeedbackMode::ShadowState || !decision.warn_timeout)
        {
            std::cerr << "Expected stale live state to fall back to shadow state and warn\n";
            return 1;
        }
    }

    std::cout << "go2_x5 arm bridge runtime test passed." << std::endl;
    return 0;
}
