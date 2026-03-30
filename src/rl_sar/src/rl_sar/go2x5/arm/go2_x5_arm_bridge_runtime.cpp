#include "rl_sar/go2x5/arm_bridge_runtime.hpp"

#include <algorithm>

#include "rl_sar/go2x5/safety_guard.hpp"

namespace Go2X5ArmBridgeRuntime
{
namespace
{

bool IsFresh(const RuntimeState& state, std::chrono::steady_clock::time_point now)
{
    Go2X5SafetyGuard::ArmBridgeStateSnapshot snapshot;
    snapshot.state_valid = state.state_valid;
    snapshot.require_live_state = state.require_live_state;
    snapshot.state_from_backend = state.state_from_backend;
    snapshot.state_timeout_sec = state.state_timeout_sec;
    snapshot.state_stamp = state.state_stamp;
    return Go2X5SafetyGuard::IsArmBridgeStateFresh(snapshot, now);
}

} // namespace

void ReconcileConfiguration(RuntimeState* state, const InitializationConfig& config)
{
    if (!state)
    {
        return;
    }

    state->joint_count = std::max(0, config.joint_count);
    state->require_state = config.require_state;
    state->require_live_state = config.require_live_state;
    state->state_timeout_sec = std::max(0.0f, config.state_timeout_sec);
    state->state_timeout_warned = false;
    state->shadow_mode_warned = false;

    const size_t joint_count = static_cast<size_t>(state->joint_count);
    if (state->external_state_q.size() != joint_count)
    {
        state->external_state_q.assign(joint_count, 0.0f);
        state->external_state_dq.assign(joint_count, 0.0f);
        state->external_state_tau.assign(joint_count, 0.0f);
        state->state_valid = false;
        state->state_from_backend = false;
    }
    if (state->shadow_q.size() != joint_count)
    {
        state->shadow_q.assign(joint_count, 0.0f);
        state->shadow_dq.assign(joint_count, 0.0f);
        if (config.command_size > 0 &&
            config.hold_position.size() == static_cast<size_t>(config.command_size))
        {
            const int count = std::min(state->joint_count, config.command_size);
            for (int i = 0; i < count; ++i)
            {
                state->shadow_q[static_cast<size_t>(i)] = config.hold_position[static_cast<size_t>(i)];
            }
        }
    }
}

bool ParseStatePayload(const std::vector<float>& data, int joint_count, StateSample* sample)
{
    if (!sample || joint_count <= 0)
    {
        return false;
    }

    const size_t n = static_cast<size_t>(joint_count);
    sample->q.assign(n, 0.0f);
    sample->dq.assign(n, 0.0f);
    sample->tau.assign(n, 0.0f);

    if (data.size() >= 3 * n)
    {
        std::copy_n(data.begin(), n, sample->q.begin());
        std::copy_n(data.begin() + n, n, sample->dq.begin());
        std::copy_n(data.begin() + 2 * n, n, sample->tau.begin());
        return true;
    }
    if (data.size() >= n)
    {
        std::copy_n(data.begin(), n, sample->q.begin());
        return true;
    }
    return false;
}

ApplyStateSampleResult ApplyStateSample(RuntimeState* state,
                                        const StateSample& sample,
                                        bool state_from_backend,
                                        std::chrono::steady_clock::time_point now)
{
    ApplyStateSampleResult result;
    if (!state || state->joint_count <= 0)
    {
        return result;
    }

    const size_t n = static_cast<size_t>(state->joint_count);
    if (sample.q.size() != n || sample.dq.size() != n || sample.tau.size() != n)
    {
        return result;
    }

    state->external_state_q = sample.q;
    state->external_state_dq = sample.dq;
    state->external_state_tau = sample.tau;
    state->state_valid = true;
    state->state_from_backend = state_from_backend;
    state->state_timeout_warned = false;
    state->state_stamp =
        (now == std::chrono::steady_clock::time_point{}) ? std::chrono::steady_clock::now() : now;
    result.accepted = true;

    if (state->state_from_backend)
    {
        state->shadow_mode_warned = false;
        if (!state->state_stream_logged)
        {
            state->state_stream_logged = true;
            result.stream_detected = true;
        }
    }
    else if (state->require_live_state && !state->shadow_mode_warned)
    {
        state->shadow_mode_warned = true;
        result.warn_shadow_only = true;
    }

    return result;
}

ReadDecision EvaluateReadDecision(const RuntimeState& state,
                                  std::chrono::steady_clock::time_point now)
{
    ReadDecision decision;
    decision.bridge_state_fresh = IsFresh(state, now);
    decision.shadow_only_state = state.require_live_state && state.state_valid && !state.state_from_backend;

    const bool use_bridge_state = decision.bridge_state_fresh &&
                                  state.external_state_q.size() >= static_cast<size_t>(state.joint_count);
    const bool has_shadow_state = state.shadow_q.size() >= static_cast<size_t>(state.joint_count) &&
                                  state.shadow_dq.size() >= static_cast<size_t>(state.joint_count);

    if (use_bridge_state)
    {
        decision.mode = FeedbackMode::LiveState;
    }
    else if (has_shadow_state)
    {
        decision.mode = FeedbackMode::ShadowState;
    }

    if (state.require_state)
    {
        if (decision.shadow_only_state)
        {
            decision.warn_shadow_only = !state.shadow_mode_warned;
        }
        else if (!decision.bridge_state_fresh)
        {
            decision.warn_timeout = !state.state_timeout_warned;
        }
        else
        {
            decision.clear_warnings = true;
        }
    }

    return decision;
}

void ApplyReadDecision(RuntimeState* state, const ReadDecision& decision)
{
    if (!state)
    {
        return;
    }

    if (decision.clear_warnings)
    {
        state->state_timeout_warned = false;
        state->shadow_mode_warned = false;
    }
    else if (decision.warn_shadow_only)
    {
        state->shadow_mode_warned = true;
    }
    else if (decision.warn_timeout)
    {
        state->state_timeout_warned = true;
    }
}

void UpdateShadowState(RuntimeState* state,
                       const std::vector<float>& q,
                       const std::vector<float>& dq)
{
    if (!state)
    {
        return;
    }
    state->shadow_q = q;
    state->shadow_dq = dq;
}

} // namespace Go2X5ArmBridgeRuntime
