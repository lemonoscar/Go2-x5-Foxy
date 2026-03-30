#include "rl_sar/go2x5/arm_bridge_transport.hpp"

#include <sstream>
#include <utility>

namespace Go2X5ArmBridgeTransport
{

namespace
{

class NullArmBridgeTransport final : public ArmBridgeTransport
{
public:
    explicit NullArmBridgeTransport(EndpointConfig config)
        : config_(std::move(config))
    {
    }

    const EndpointConfig& config() const override
    {
        return this->config_;
    }

    Backend backend() const override
    {
        return this->config_.backend;
    }

    std::string Describe() const override
    {
        std::ostringstream oss;
        oss << "arm-bridge-null[" << BackendName(this->backend())
            << ", transport=" << NormalizeTransportName(this->config_.transport)
            << ", joint_count=" << this->config_.joint_count << "]";
        return oss.str();
    }

    bool SendCommand(const CommandFrame&, std::string* error = nullptr) override
    {
        if (error)
        {
            *error = "arm bridge transport not wired: " + this->Describe();
        }
        return false;
    }

    bool TryReceiveState(StateFrame*, std::string* error = nullptr) override
    {
        if (error)
        {
            *error = "arm bridge transport not wired: " + this->Describe();
        }
        return false;
    }

private:
    EndpointConfig config_;
};

} // namespace

std::string NormalizeTransportName(std::string transport)
{
    transport = Go2X5IPC::NormalizeTransport(std::move(transport));
    if (transport.empty())
    {
        transport = "ros";
    }
    return transport;
}

Backend ParseBackend(std::string transport)
{
    transport = NormalizeTransportName(std::move(transport));
    return Go2X5IPC::IsIpcTransport(transport) ? Backend::Ipc : Backend::Ros;
}

std::string BackendName(Backend backend)
{
    return backend == Backend::Ipc ? "ipc" : "ros";
}

FrameSummary Summarize(const StateFrame& frame)
{
    FrameSummary summary;
    summary.transport = frame.source;
    summary.state_from_backend = frame.state_from_backend;
    summary.shadow_only = frame.shadow_only;
    summary.valid = frame.valid;
    summary.has_freshness_stamp = frame.has_received_at;
    summary.joint_count = frame.joint_count;
    return summary;
}

bool IsShadowOnly(const StateFrame& frame)
{
    return frame.IsShadowOnly();
}

Go2X5IPC::ArmCommandPacket ToIpcCommandPacket(const CommandFrame& frame)
{
    Go2X5IPC::ArmCommandPacket packet;
    packet.joint_count = frame.joint_count;
    packet.q = frame.q;
    packet.dq = frame.dq;
    packet.kp = frame.kp;
    packet.kd = frame.kd;
    packet.tau = frame.tau;
    return packet;
}

Go2X5IPC::ArmStatePacket ToIpcStatePacket(const StateFrame& frame)
{
    Go2X5IPC::ArmStatePacket packet;
    packet.joint_count = frame.joint_count;
    packet.state_from_backend = frame.state_from_backend;
    packet.q = frame.q;
    packet.dq = frame.dq;
    packet.tau = frame.tau;
    return packet;
}

CommandFrame FromIpcCommandPacket(const Go2X5IPC::ArmCommandPacket& packet)
{
    CommandFrame frame = MakeCommandFrame(packet.joint_count);
    frame.q = packet.q;
    frame.dq = packet.dq;
    frame.kp = packet.kp;
    frame.kd = packet.kd;
    frame.tau = packet.tau;
    return frame;
}

StateFrame FromIpcStatePacket(
    const Go2X5IPC::ArmStatePacket& packet,
    std::chrono::steady_clock::time_point received_at)
{
    StateFrame frame = MakeStateFrame(packet.joint_count);
    frame.state_from_backend = packet.state_from_backend;
    frame.shadow_only = !packet.state_from_backend;
    frame.valid = true;
    frame.received_at = received_at;
    frame.has_received_at = received_at.time_since_epoch() != std::chrono::steady_clock::duration::zero();
    frame.source = "ipc";
    frame.q = packet.q;
    frame.dq = packet.dq;
    frame.tau = packet.tau;
    return frame;
}

std::unique_ptr<ArmBridgeTransport> CreateArmBridgeTransport(
    const EndpointConfig& config,
    std::string* error)
{
    if (config.joint_count == 0)
    {
        if (error)
        {
            *error = "arm bridge transport requires a non-zero joint_count";
        }
        return nullptr;
    }

    return std::unique_ptr<ArmBridgeTransport>(new NullArmBridgeTransport(config));
}

} // namespace Go2X5ArmBridgeTransport
