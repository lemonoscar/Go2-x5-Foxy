#ifndef GO2_X5_ARM_BRIDGE_RUNTIME_HPP
#define GO2_X5_ARM_BRIDGE_RUNTIME_HPP

#include <chrono>
#include <vector>

namespace Go2X5ArmBridgeRuntime
{

enum class FeedbackMode
{
    None,
    LiveState,
    ShadowState,
};

struct RuntimeState
{
    int joint_count = 0;
    bool require_state = true;
    bool require_live_state = true;
    float state_timeout_sec = 0.25f;
    bool state_valid = false;
    bool state_from_backend = false;
    bool state_timeout_warned = false;
    bool shadow_mode_warned = false;
    bool state_stream_logged = false;
    std::chrono::steady_clock::time_point state_stamp{};
    std::vector<float> external_state_q;
    std::vector<float> external_state_dq;
    std::vector<float> external_state_tau;
    std::vector<float> shadow_q;
    std::vector<float> shadow_dq;
};

struct InitializationConfig
{
    int joint_count = 0;
    int command_size = 0;
    bool require_state = true;
    bool require_live_state = true;
    float state_timeout_sec = 0.25f;
    std::vector<float> hold_position;
};

struct StateSample
{
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
};

struct ApplyStateSampleResult
{
    bool accepted = false;
    bool stream_detected = false;
    bool warn_shadow_only = false;
};

struct ReadDecision
{
    FeedbackMode mode = FeedbackMode::None;
    bool bridge_state_fresh = false;
    bool shadow_only_state = false;
    bool warn_shadow_only = false;
    bool warn_timeout = false;
    bool clear_warnings = false;
};

void ReconcileConfiguration(RuntimeState* state, const InitializationConfig& config);
bool ParseStatePayload(const std::vector<float>& data, int joint_count, StateSample* sample);
ApplyStateSampleResult ApplyStateSample(RuntimeState* state,
                                        const StateSample& sample,
                                        bool state_from_backend,
                                        std::chrono::steady_clock::time_point now =
                                            std::chrono::steady_clock::time_point{});
ReadDecision EvaluateReadDecision(const RuntimeState& state,
                                  std::chrono::steady_clock::time_point now =
                                      std::chrono::steady_clock::time_point{});
void ApplyReadDecision(RuntimeState* state, const ReadDecision& decision);
void UpdateShadowState(RuntimeState* state,
                       const std::vector<float>& q,
                       const std::vector<float>& dq);

} // namespace Go2X5ArmBridgeRuntime

#endif // GO2_X5_ARM_BRIDGE_RUNTIME_HPP
