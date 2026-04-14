#ifndef RL_SAR_TRAJECTORY_FALLBACK_SMOOTHER_HPP
#define RL_SAR_TRAJECTORY_FALLBACK_SMOOTHER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "rl_sar/protocol/go2_x5_protocol.hpp"

namespace rl_sar::trajectory
{

struct FallbackSmootherConfig
{
    float max_joint_velocity = 3.14f;
    float default_duration = 0.5f;
    float min_duration = 0.2f;
    float max_duration = 2.0f;
};

class FallbackSmoother
{
public:
    explicit FallbackSmoother(FallbackSmootherConfig config = {})
        : config_(config)
    {
    }

    void Configure(const FallbackSmootherConfig& config)
    {
        config_ = config;
    }

    void Plan(const std::array<float, protocol::kBodyJointCount>& current_q,
              const std::array<float, protocol::kBodyJointCount>& target_q,
              uint64_t start_time_ns,
              float duration_sec = 0.0f)
    {
        trajectory_.start_q = current_q;
        trajectory_.target_q = target_q;
        trajectory_.start_time_ns = start_time_ns;
        trajectory_.active = true;

        if (duration_sec <= 0.0f)
        {
            duration_sec = ComputeMinDuration(current_q, target_q);
            duration_sec = std::max(duration_sec, config_.default_duration);
        }

        duration_sec = std::clamp(duration_sec, config_.min_duration, config_.max_duration);
        trajectory_.duration_ns = static_cast<uint64_t>(duration_sec * 1e9f);
    }

    void Reset()
    {
        trajectory_ = Trajectory{};
    }

    bool IsActive(uint64_t now_ns) const
    {
        return trajectory_.active && now_ns < (trajectory_.start_time_ns + trajectory_.duration_ns);
    }

    bool IsCompleted(uint64_t now_ns) const
    {
        return !trajectory_.active ||
               now_ns >= (trajectory_.start_time_ns + trajectory_.duration_ns);
    }

    float GetProgress(uint64_t now_ns) const
    {
        if (!trajectory_.active || trajectory_.duration_ns == 0)
        {
            return 1.0f;
        }
        if (now_ns <= trajectory_.start_time_ns)
        {
            return 0.0f;
        }
        const uint64_t elapsed_ns = now_ns - trajectory_.start_time_ns;
        if (elapsed_ns >= trajectory_.duration_ns)
        {
            return 1.0f;
        }
        return static_cast<float>(elapsed_ns) / static_cast<float>(trajectory_.duration_ns);
    }

    std::array<float, protocol::kBodyJointCount> GetTargetPosition(uint64_t now_ns) const
    {
        std::array<float, protocol::kBodyJointCount> q = trajectory_.target_q;
        if (!trajectory_.active)
        {
            return q;
        }

        const float s = ComputeInterpolationFactor(now_ns);
        for (size_t i = 0; i < q.size(); ++i)
        {
            q[i] = trajectory_.start_q[i] +
                   (trajectory_.target_q[i] - trajectory_.start_q[i]) * s;
        }
        return q;
    }

    std::array<float, protocol::kBodyJointCount> GetTargetVelocity(uint64_t now_ns) const
    {
        std::array<float, protocol::kBodyJointCount> dq{};
        if (!trajectory_.active || trajectory_.duration_ns == 0)
        {
            return dq;
        }

        if (now_ns <= trajectory_.start_time_ns)
        {
            return dq;
        }

        const uint64_t elapsed_ns = now_ns - trajectory_.start_time_ns;
        if (elapsed_ns >= trajectory_.duration_ns)
        {
            return dq;
        }

        const float T = static_cast<float>(trajectory_.duration_ns) / 1e9f;
        if (T <= 0.0f)
        {
            return dq;
        }

        const float tau = std::clamp(
            static_cast<float>(elapsed_ns) / static_cast<float>(trajectory_.duration_ns),
            0.0f,
            1.0f);
        const float ds_dt =
            (30.0f * tau * tau - 60.0f * tau * tau * tau + 30.0f * tau * tau * tau * tau) / T;

        for (size_t i = 0; i < dq.size(); ++i)
        {
            dq[i] = (trajectory_.target_q[i] - trajectory_.start_q[i]) * ds_dt;
        }
        return dq;
    }

private:
    struct Trajectory
    {
        std::array<float, protocol::kBodyJointCount> start_q{};
        std::array<float, protocol::kBodyJointCount> target_q{};
        uint64_t start_time_ns = 0;
        uint64_t duration_ns = 0;
        bool active = false;
    };

    float ComputeMinDuration(
        const std::array<float, protocol::kBodyJointCount>& current_q,
        const std::array<float, protocol::kBodyJointCount>& target_q) const
    {
        float duration_sec = config_.min_duration;
        const float max_joint_velocity = std::max(config_.max_joint_velocity, 1e-3f);
        for (size_t i = 0; i < current_q.size(); ++i)
        {
            const float diff = std::fabs(target_q[i] - current_q[i]);
            duration_sec = std::max(duration_sec, 1.9f * diff / max_joint_velocity);
        }
        return duration_sec;
    }

    float ComputeInterpolationFactor(uint64_t now_ns) const
    {
        const float tau = std::clamp(GetProgress(now_ns), 0.0f, 1.0f);
        return 10.0f * tau * tau * tau -
               15.0f * tau * tau * tau * tau +
               6.0f * tau * tau * tau * tau * tau;
    }

    FallbackSmootherConfig config_{};
    Trajectory trajectory_{};
};

}  // namespace rl_sar::trajectory

#endif  // RL_SAR_TRAJECTORY_FALLBACK_SMOOTHER_HPP
