#ifndef GO2_X5_ARM_TRANSPORT_HPP
#define GO2_X5_ARM_TRANSPORT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Go2X5ArmTransport {

/**
 * @brief Arm command structure
 *
 * Contains the full command to send to the arm bridge.
 */
struct ArmCommand {
    std::vector<float> q;   // Joint positions (rad)
    std::vector<float> dq;  // Joint velocities (rad/s)
    std::vector<float> kp;  // Position gains
    std::vector<float> kd;  // Damping gains
    std::vector<float> tau; // Feedforward torques (Nm)

    size_t joint_count() const { return q.size(); }

    bool is_valid() const {
        return q.size() == dq.size() &&
               q.size() == kp.size() &&
               q.size() == kd.size() &&
               q.size() == tau.size();
    }
};

/**
 * @brief Arm state structure
 *
 * Contains the full state received from the arm bridge.
 */
struct ArmState {
    std::vector<float> q;   // Joint positions (rad)
    std::vector<float> dq;  // Joint velocities (rad/s)
    std::vector<float> tau; // Joint torques (Nm)
    bool from_backend = false;  // True if state comes from ARX backend
    std::chrono::steady_clock::time_point stamp;

    size_t joint_count() const { return q.size(); }

    bool is_valid() const {
        return q.size() == dq.size() &&
               q.size() == tau.size();
    }

    double age_ms() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - stamp).count();
    }
};

/**
 * @brief Transport mode enumeration
 */
enum class TransportMode {
    IPC    // Use IPC (UDP sockets)
};

/**
 * @brief Transport statistics for monitoring
 */
struct TransportStats {
    uint64_t commands_sent = 0;
    uint64_t commands_failed = 0;
    uint64_t states_received = 0;
    uint64_t states_timeout = 0;
    double last_send_latency_ms = 0.0;
    double last_receive_latency_ms = 0.0;

    double success_rate() const {
        uint64_t total = commands_sent + commands_failed;
        return total > 0 ? static_cast<double>(commands_sent) / total : 0.0;
    }
};

/**
 * @brief Abstract transport interface for arm bridge communication
 *
 * This interface abstracts the underlying transport mechanism (IPC)
 * for communicating with the arm bridge.
 */
class Transport {
public:
    virtual ~Transport() = default;

    /**
     * @brief Initialize the transport
     * @return true if initialization successful
     */
    virtual bool Initialize(std::string* error) = 0;

    /**
     * @brief Send arm command to the bridge
     * @param cmd The command to send
     * @param error Output parameter for error message
     * @return true if send successful
     */
    virtual bool SendCommand(const ArmCommand& cmd, std::string* error = nullptr) = 0;

    /**
     * @brief Receive arm state from the bridge (non-blocking)
     * @param state Output parameter for received state
     * @param error Output parameter for error message
     * @return true if state was received, false if no new state available
     */
    virtual bool ReceiveState(ArmState* state, std::string* error = nullptr) = 0;

    /**
     * @brief Check if transport is connected/active
     * @return true if connected
     */
    virtual bool IsConnected() const = 0;

    /**
     * @brief Shutdown the transport
     */
    virtual void Shutdown() = 0;

    /**
     * @brief Get transport statistics
     * @return Current transport statistics
     */
    virtual TransportStats GetStats() const = 0;

    /**
     * @brief Reset statistics counters
     */
    virtual void ResetStats() = 0;

    /**
     * @brief Get transport mode
     * @return The transport mode
     */
    virtual TransportMode GetMode() const = 0;
};

/**
 * @brief Factory for creating transport instances
 */
class TransportFactory {
public:
    /**
     * @brief Create an IPC transport
     * @param cmd_port UDP port for sending commands
     * @param state_port UDP port for receiving states
     * @param host IP address (default: 127.0.0.1)
     * @return Transport instance or nullptr on failure
     */
    static std::unique_ptr<Transport> CreateIpcTransport(
        int cmd_port = 45671,
        int state_port = 45672,
        const std::string& host = "127.0.0.1"
    );
};

/**
 * @brief IPC transport implementation using UDP sockets
 *
 * Uses localhost UDP for arm bridge communication.
 */
class IpcTransport : public Transport {
public:
    IpcTransport(int cmd_port, int state_port, const std::string& host);

    ~IpcTransport() override;

    // Transport interface
    bool Initialize(std::string* error) override;
    bool SendCommand(const ArmCommand& cmd, std::string* error = nullptr) override;
    bool ReceiveState(ArmState* state, std::string* error = nullptr) override;
    bool IsConnected() const override;
    void Shutdown() override;
    TransportStats GetStats() const override;
    void ResetStats() override;
    TransportMode GetMode() const override { return TransportMode::IPC; }

private:
    bool CreateSockets(std::string* error);
    void CloseSockets();

    int cmd_port_;
    int state_port_;
    std::string host_;

    int cmd_socket_fd_ = -1;
    int state_socket_fd_ = -1;

    ArmState last_state_;
    mutable std::mutex state_mutex_;

    TransportStats stats_;
    bool initialized_ = false;
};

/**
 * @brief Smart transport selector
 *
 * Automatically selects IPC transport for arm bridge communication.
 */
class SmartTransport {
public:
    SmartTransport();

    /**
     * @brief Initialize with IPC transport
     * @return true if initialization successful
     */
    bool Initialize(bool prefer_ipc = false);

    /**
     * @brief Get the active transport
     * @return Pointer to the active transport (never null after successful init)
     */
    Transport* GetTransport();

    /**
     * @brief Get the active transport (const)
     * @return Pointer to the active transport (never null after successful init)
     */
    const Transport* GetTransport() const;

    /**
     * @brief Check if IPC mode is active
     * @return true if IPC transport is being used
     */
    bool IsIpcMode() const;

    /**
     * @brief Check if ROS mode is active (always false in CMake-only mode)
     * @return true if ROS transport is being used
     */
    bool IsRosMode() const;

    /**
     * @brief Shutdown the transport
     */
    void Shutdown();

private:
    std::unique_ptr<Transport> transport_;
    TransportMode active_mode_;
};

} // namespace Go2X5ArmTransport

#endif // GO2_X5_ARM_TRANSPORT_HPP
