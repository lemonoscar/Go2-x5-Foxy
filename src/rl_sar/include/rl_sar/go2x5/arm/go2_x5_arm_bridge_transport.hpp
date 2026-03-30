#ifndef GO2_X5_ARM_BRIDGE_TRANSPORT_HPP
#define GO2_X5_ARM_BRIDGE_TRANSPORT_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rl_sar/go2x5/comm/go2_x5_ipc_protocol.hpp"

namespace Go2X5ArmBridgeTransport
{

constexpr const char* kDefaultRosCommandTopic = "/arx_x5/joint_cmd";
constexpr const char* kDefaultRosStateTopic = "/arx_x5/joint_state";
constexpr const char* kDefaultJointCommandTopic = "/arm_joint_pos_cmd";

enum class Backend
{
    Ros,
    Ipc
};

struct EndpointConfig
{
    Backend backend = Backend::Ros;
    std::string transport = "ros";
    std::string command_topic = kDefaultRosCommandTopic;
    std::string state_topic = kDefaultRosStateTopic;
    std::string joint_command_topic = kDefaultJointCommandTopic;
    std::string ipc_host = Go2X5IPC::kDefaultHost;
    int ipc_cmd_port = Go2X5IPC::kDefaultCommandPort;
    int ipc_state_port = Go2X5IPC::kDefaultStatePort;
    int joint_command_port = Go2X5IPC::kDefaultJointCommandPort;
    uint16_t joint_count = 0;
    bool accept_commands = true;
    bool require_state = true;
    bool require_live_state = true;
};

struct CommandFrame
{
    uint16_t joint_count = 0;
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> kp;
    std::vector<float> kd;
    std::vector<float> tau;
};

struct StateFrame
{
    uint16_t joint_count = 0;
    bool state_from_backend = false;
    bool shadow_only = false;
    bool valid = false;
    bool has_received_at = false;
    std::chrono::steady_clock::time_point received_at{};
    std::string source;
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;

    bool HasFreshnessStamp() const
    {
        return this->has_received_at;
    }

    bool IsShadowOnly() const
    {
        return this->valid && this->shadow_only;
    }
};

inline CommandFrame MakeCommandFrame(const uint16_t joint_count)
{
    CommandFrame frame;
    frame.joint_count = joint_count;
    frame.q.assign(static_cast<size_t>(joint_count), 0.0f);
    frame.dq.assign(static_cast<size_t>(joint_count), 0.0f);
    frame.kp.assign(static_cast<size_t>(joint_count), 0.0f);
    frame.kd.assign(static_cast<size_t>(joint_count), 0.0f);
    frame.tau.assign(static_cast<size_t>(joint_count), 0.0f);
    return frame;
}

inline StateFrame MakeStateFrame(const uint16_t joint_count)
{
    StateFrame frame;
    frame.joint_count = joint_count;
    frame.q.assign(static_cast<size_t>(joint_count), 0.0f);
    frame.dq.assign(static_cast<size_t>(joint_count), 0.0f);
    frame.tau.assign(static_cast<size_t>(joint_count), 0.0f);
    return frame;
}

std::string NormalizeTransportName(std::string transport);
Backend ParseBackend(std::string transport);
std::string BackendName(Backend backend);

struct FrameSummary
{
    std::string transport;
    bool state_from_backend = false;
    bool shadow_only = false;
    bool valid = false;
    bool has_freshness_stamp = false;
    uint16_t joint_count = 0;
};

FrameSummary Summarize(const StateFrame& frame);
bool IsShadowOnly(const StateFrame& frame);

Go2X5IPC::ArmCommandPacket ToIpcCommandPacket(const CommandFrame& frame);
Go2X5IPC::ArmStatePacket ToIpcStatePacket(const StateFrame& frame);
CommandFrame FromIpcCommandPacket(const Go2X5IPC::ArmCommandPacket& packet);
StateFrame FromIpcStatePacket(
    const Go2X5IPC::ArmStatePacket& packet,
    std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::time_point{});

class ArmBridgeTransport
{
public:
    virtual ~ArmBridgeTransport() = default;

    virtual const EndpointConfig& config() const = 0;
    virtual Backend backend() const = 0;
    virtual std::string Describe() const = 0;

    virtual bool SendCommand(const CommandFrame& frame, std::string* error = nullptr) = 0;
    virtual bool TryReceiveState(StateFrame* frame, std::string* error = nullptr) = 0;
};

std::unique_ptr<ArmBridgeTransport> CreateArmBridgeTransport(
    const EndpointConfig& config,
    std::string* error = nullptr);

} // namespace Go2X5ArmBridgeTransport

#endif // GO2_X5_ARM_BRIDGE_TRANSPORT_HPP
