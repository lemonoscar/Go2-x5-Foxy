#include "rl_sar/go2x5/safety_guard.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "logger.hpp"

namespace Go2X5SafetyGuard
{
namespace
{

float DefaultFallbackValue(const ArmSafetyContext& safety_context, size_t index)
{
    if (index < safety_context.arm_joint_lower_limits.size() &&
        index < safety_context.arm_joint_upper_limits.size())
    {
        const float lo = std::min(safety_context.arm_joint_lower_limits[index],
                                  safety_context.arm_joint_upper_limits[index]);
        const float hi = std::max(safety_context.arm_joint_lower_limits[index],
                                  safety_context.arm_joint_upper_limits[index]);
        return std::clamp(0.0f, lo, hi);
    }
    return 0.0f;
}

float ClipAbs(float value, float fallback, float abs_limit, bool non_negative, bool* clipped)
{
    float sanitized = std::isfinite(value) ? value : fallback;
    if (std::isfinite(abs_limit))
    {
        const float lo = non_negative ? 0.0f : -std::fabs(abs_limit);
        const float hi = std::fabs(abs_limit);
        const float clamped = std::clamp(sanitized, lo, hi);
        if (std::fabs(clamped - sanitized) > 1e-6f || !std::isfinite(value))
        {
            *clipped = true;
        }
        return clamped;
    }
    if (!std::isfinite(value))
    {
        *clipped = true;
    }
    return sanitized;
}

} // namespace

bool ClipArmPoseTargetInPlace(std::vector<float>* target,
                              const std::vector<float>& fallback,
                              const ArmSafetyContext& safety_context,
                              const char* context)
{
    if (!target)
    {
        return false;
    }
    if (target->size() != static_cast<size_t>(safety_context.arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: expect "
                  << safety_context.arm_joint_count << " values, got " << target->size() << std::endl;
        return false;
    }

    bool clipped = false;
    for (size_t i = 0; i < target->size(); ++i)
    {
        float value = (*target)[i];
        const float fallback_value =
            (i < fallback.size()) ? fallback[i] : DefaultFallbackValue(safety_context, i);
        if (!std::isfinite(value))
        {
            value = fallback_value;
            clipped = true;
        }
        if (i < safety_context.arm_joint_lower_limits.size() &&
            i < safety_context.arm_joint_upper_limits.size())
        {
            const float lo = std::min(safety_context.arm_joint_lower_limits[i],
                                      safety_context.arm_joint_upper_limits[i]);
            const float hi = std::max(safety_context.arm_joint_lower_limits[i],
                                      safety_context.arm_joint_upper_limits[i]);
            const float clamped = std::clamp(value, lo, hi);
            if (std::fabs(clamped - value) > 1e-6f)
            {
                clipped = true;
            }
            value = clamped;
        }
        (*target)[i] = value;
    }

    if (clipped)
    {
        std::cout << LOGGER::WARNING << context << " clipped to arm joint limits." << std::endl;
    }
    return true;
}

bool ClipArmBridgeCommandInPlace(std::vector<float>* q,
                                 std::vector<float>* dq,
                                 std::vector<float>* kp,
                                 std::vector<float>* kd,
                                 std::vector<float>* tau,
                                 const std::vector<float>& q_fallback,
                                 const ArmSafetyContext& safety_context,
                                 const char* context)
{
    if (!q || !dq || !kp || !kd || !tau)
    {
        return false;
    }
    if (q->size() != static_cast<size_t>(safety_context.arm_joint_count) ||
        dq->size() != static_cast<size_t>(safety_context.arm_joint_count) ||
        kp->size() != static_cast<size_t>(safety_context.arm_joint_count) ||
        kd->size() != static_cast<size_t>(safety_context.arm_joint_count) ||
        tau->size() != static_cast<size_t>(safety_context.arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: arm bridge command size mismatch" << std::endl;
        return false;
    }

    if (!ClipArmPoseTargetInPlace(q, q_fallback, safety_context, context))
    {
        return false;
    }

    bool clipped = false;
    for (int i = 0; i < safety_context.arm_joint_count; ++i)
    {
        const int idx = safety_context.arm_joint_start_index + i;
        const float dq_limit =
            (idx >= 0 && idx < static_cast<int>(safety_context.whole_body_velocity_limits.size()))
                ? safety_context.whole_body_velocity_limits[static_cast<size_t>(idx)]
                : 3.0f;
        const float kp_limit =
            (idx >= 0 && idx < static_cast<int>(safety_context.whole_body_kp_limits.size()))
                ? safety_context.whole_body_kp_limits[static_cast<size_t>(idx)]
                : 100.0f;
        const float kd_limit =
            (idx >= 0 && idx < static_cast<int>(safety_context.whole_body_kd_limits.size()))
                ? safety_context.whole_body_kd_limits[static_cast<size_t>(idx)]
                : 20.0f;
        const float tau_limit =
            (idx >= 0 && idx < static_cast<int>(safety_context.whole_body_effort_limits.size()))
                ? safety_context.whole_body_effort_limits[static_cast<size_t>(idx)]
                : 15.0f;

        (*dq)[static_cast<size_t>(i)] = ClipAbs((*dq)[static_cast<size_t>(i)], 0.0f, dq_limit, false, &clipped);
        (*kp)[static_cast<size_t>(i)] = ClipAbs((*kp)[static_cast<size_t>(i)], 0.0f, kp_limit, true, &clipped);
        (*kd)[static_cast<size_t>(i)] = ClipAbs((*kd)[static_cast<size_t>(i)], 0.0f, kd_limit, true, &clipped);
        (*tau)[static_cast<size_t>(i)] = ClipAbs((*tau)[static_cast<size_t>(i)], 0.0f, tau_limit, false, &clipped);
    }

    if (clipped)
    {
        std::cout << LOGGER::WARNING << context << " clipped to real arm deploy limits." << std::endl;
    }
    return true;
}

bool ValidateArmPoseTarget(const std::vector<float>& target,
                           const ArmSafetyContext& safety_context,
                           const char* context)
{
    if (target.size() != static_cast<size_t>(safety_context.arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: expect "
                  << safety_context.arm_joint_count << " values, got " << target.size() << std::endl;
        return false;
    }

    for (size_t i = 0; i < target.size(); ++i)
    {
        const float value = target[i];
        if (!std::isfinite(value))
        {
            std::cout << LOGGER::WARNING << context << " rejected: non-finite joint["
                      << i << "]=" << value << std::endl;
            return false;
        }
        if (i < safety_context.arm_joint_lower_limits.size() && value < safety_context.arm_joint_lower_limits[i])
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << value << " below lower limit "
                      << safety_context.arm_joint_lower_limits[i] << std::endl;
            return false;
        }
        if (i < safety_context.arm_joint_upper_limits.size() && value > safety_context.arm_joint_upper_limits[i])
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << value << " above upper limit "
                      << safety_context.arm_joint_upper_limits[i] << std::endl;
            return false;
        }
    }
    return true;
}

bool ValidateArmBridgeStateSample(const std::vector<float>& q,
                                  const std::vector<float>& dq,
                                  const std::vector<float>& tau,
                                  const ArmSafetyContext& safety_context,
                                  const char* context)
{
    if (q.size() != static_cast<size_t>(safety_context.arm_joint_count) ||
        dq.size() != static_cast<size_t>(safety_context.arm_joint_count) ||
        tau.size() != static_cast<size_t>(safety_context.arm_joint_count))
    {
        std::cout << LOGGER::WARNING << context << " rejected: state size mismatch" << std::endl;
        return false;
    }

    constexpr float kStateMargin = 0.35f;
    for (size_t i = 0; i < q.size(); ++i)
    {
        if (!std::isfinite(q[i]) || !std::isfinite(dq[i]) || !std::isfinite(tau[i]))
        {
            std::cout << LOGGER::WARNING << context << " rejected: non-finite state sample at joint "
                      << i << std::endl;
            return false;
        }
        if (i < safety_context.arm_joint_lower_limits.size() &&
            q[i] < (safety_context.arm_joint_lower_limits[i] - kStateMargin))
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << q[i] << " below plausible state lower bound "
                      << (safety_context.arm_joint_lower_limits[i] - kStateMargin) << std::endl;
            return false;
        }
        if (i < safety_context.arm_joint_upper_limits.size() &&
            q[i] > (safety_context.arm_joint_upper_limits[i] + kStateMargin))
        {
            std::cout << LOGGER::WARNING << context << " rejected: joint[" << i
                      << "]=" << q[i] << " above plausible state upper bound "
                      << (safety_context.arm_joint_upper_limits[i] + kStateMargin) << std::endl;
            return false;
        }
    }
    return true;
}

bool IsArmBridgeStateFresh(const ArmBridgeStateSnapshot& state_snapshot,
                           std::chrono::steady_clock::time_point now)
{
    if (!state_snapshot.state_valid)
    {
        return false;
    }
    if (state_snapshot.require_live_state && !state_snapshot.state_from_backend)
    {
        return false;
    }
    if (state_snapshot.state_timeout_sec <= 1e-6f)
    {
        return true;
    }
    if (now == std::chrono::steady_clock::time_point{})
    {
        now = std::chrono::steady_clock::now();
    }

    const auto age = std::chrono::duration_cast<std::chrono::duration<float>>(
        now - state_snapshot.state_stamp).count();
    return age <= state_snapshot.state_timeout_sec;
}

} // namespace Go2X5SafetyGuard
