#include "rl_sar/go2x5/arm_transport.hpp"
#include "rl_sar/go2x5/ipc_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace Go2X5ArmTransport {

// ============================================================================
// IpcTransport Implementation
// ============================================================================

IpcTransport::IpcTransport(int cmd_port, int state_port, const std::string& host)
    : cmd_port_(cmd_port)
    , state_port_(state_port)
    , host_(host) {}

IpcTransport::~IpcTransport() {
    Shutdown();
}

bool IpcTransport::Initialize(std::string* error) {
    return CreateSockets(error);
}

bool IpcTransport::CreateSockets(std::string* error) {
    // Create command socket (UDP)
    cmd_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_socket_fd_ < 0) {
        if (error) *error = "Failed to create command socket: " + std::string(strerror(errno));
        return false;
    }

    // Create state socket (UDP)
    state_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (state_socket_fd_ < 0) {
        if (error) *error = "Failed to create state socket: " + std::string(strerror(errno));
        CloseSockets();
        return false;
    }

    // Bind state socket to receive port
    sockaddr_in state_addr{};
    std::memset(&state_addr, 0, sizeof(state_addr));
    state_addr.sin_family = AF_INET;
    state_addr.sin_addr.s_addr = INADDR_ANY;
    state_addr.sin_port = htons(state_port_);

    if (bind(state_socket_fd_, reinterpret_cast<sockaddr*>(&state_addr), sizeof(state_addr)) < 0) {
        if (error) *error = "Failed to bind state socket: " + std::string(strerror(errno));
        CloseSockets();
        return false;
    }

    // Set non-blocking mode
    int flags = fcntl(state_socket_fd_, F_GETFL, 0);
    if (flags == -1 || fcntl(state_socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
        if (error) *error = "Failed to set non-blocking mode: " + std::string(strerror(errno));
        CloseSockets();
        return false;
    }

    initialized_ = true;
    return true;
}

void IpcTransport::CloseSockets() {
    if (cmd_socket_fd_ >= 0) {
        close(cmd_socket_fd_);
        cmd_socket_fd_ = -1;
    }
    if (state_socket_fd_ >= 0) {
        close(state_socket_fd_);
        state_socket_fd_ = -1;
    }
}

bool IpcTransport::SendCommand(const ArmCommand& cmd, std::string* error) {
    if (!initialized_) {
        if (error) *error = "Transport not initialized";
        ++stats_.commands_failed;
        return false;
    }

    if (!cmd.is_valid()) {
        if (error) *error = "Invalid command: vector size mismatch";
        ++stats_.commands_failed;
        return false;
    }

    // Serialize command using IPC protocol
    auto packet = Go2X5IPC::SerializeCommandPacket(
        static_cast<uint16_t>(cmd.joint_count()),
        cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau);

    // Send to localhost
    sockaddr_in addr{};
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cmd_port_);

    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
        if (error) *error = "Invalid host address: " + host_;
        ++stats_.commands_failed;
        return false;
    }

    auto start = std::chrono::steady_clock::now();

    ssize_t sent = sendto(cmd_socket_fd_, packet.data(), packet.size(), 0,
                          reinterpret_cast<sockaddr*>(&addr), sizeof(addr));

    auto end = std::chrono::steady_clock::now();
    stats_.last_send_latency_ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (sent < 0) {
        if (error) *error = "Failed to send command: " + std::string(strerror(errno));
        ++stats_.commands_failed;
        return false;
    }

    if (static_cast<size_t>(sent) != packet.size()) {
        if (error) *error = "Partial send: " + std::to_string(sent) + " / " + std::to_string(packet.size());
        ++stats_.commands_failed;
        return false;
    }

    ++stats_.commands_sent;
    return true;
}

bool IpcTransport::ReceiveState(ArmState* state, std::string* error) {
    if (!initialized_) {
        if (error) *error = "Transport not initialized";
        return false;
    }

    // Try to receive state packet
    std::vector<uint8_t> buffer(4096);
    sockaddr_in from_addr{};
    socklen_t from_len = sizeof(from_addr);

    ssize_t received = recvfrom(state_socket_fd_, buffer.data(), buffer.size(), 0,
                                reinterpret_cast<sockaddr*>(&from_addr), &from_len);

    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // No data available (non-blocking)
        }
        if (error) *error = "Failed to receive state: " + std::string(strerror(errno));
        return false;
    }

    buffer.resize(static_cast<size_t>(received));

    // Parse state packet
    Go2X5IPC::ArmStatePacket ipc_state;
    std::string parse_error;
    if (!Go2X5IPC::ParseStatePacket(buffer, ipc_state, &parse_error)) {
        if (error) *error = "Failed to parse state packet: " + parse_error;
        return false;
    }

    ArmState result;
    result.q = ipc_state.q;
    result.dq = ipc_state.dq;
    result.tau = ipc_state.tau;
    result.from_backend = ipc_state.state_from_backend;
    result.stamp = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_state_ = std::move(result);
    }

    if (state) {
        *state = last_state_;
        ++stats_.states_received;
    }

    return true;
}

bool IpcTransport::IsConnected() const {
    return initialized_ && cmd_socket_fd_ >= 0 && state_socket_fd_ >= 0;
}

void IpcTransport::Shutdown() {
    CloseSockets();
    initialized_ = false;
}

TransportStats IpcTransport::GetStats() const {
    return stats_;
}

void IpcTransport::ResetStats() {
    stats_ = TransportStats();
}

// ============================================================================
// TransportFactory Implementation
// ============================================================================

std::unique_ptr<Transport> TransportFactory::CreateIpcTransport(
    int cmd_port,
    int state_port,
    const std::string& host) {
    auto transport = std::make_unique<IpcTransport>(cmd_port, state_port, host);
    return transport;
}

// ============================================================================
// SmartTransport Implementation
// ============================================================================

SmartTransport::SmartTransport()
    : active_mode_(TransportMode::IPC) {}

bool SmartTransport::Initialize(bool prefer_ipc) {
    auto ipc = TransportFactory::CreateIpcTransport();
    std::string error;
    if (ipc->Initialize(&error)) {
        transport_ = std::move(ipc);
        active_mode_ = TransportMode::IPC;
        return true;
    }
    return false;
}

Transport* SmartTransport::GetTransport() {
    return transport_.get();
}

const Transport* SmartTransport::GetTransport() const {
    return transport_.get();
}

bool SmartTransport::IsIpcMode() const {
    return active_mode_ == TransportMode::IPC;
}

bool SmartTransport::IsRosMode() const {
    return false;  // Always false in CMake-only mode
}

void SmartTransport::Shutdown() {
    if (transport_) {
        transport_->Shutdown();
        transport_.reset();
    }
}

} // namespace Go2X5ArmTransport
