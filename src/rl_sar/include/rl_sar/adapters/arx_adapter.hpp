#ifndef RL_SAR_ADAPTERS_ARX_ADAPTER_HPP
#define RL_SAR_ADAPTERS_ARX_ADAPTER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

namespace rl_sar::adapters
{

/**
 * @brief ARX X5 robotic arm adapter
 *
 * This adapter provides a clean interface to the ARX X5 robotic arm (6 DOF),
 * converging the scattered ARX SDK calls from arx_x5_bridge.py into a
 * well-structured C++ implementation.
 *
 * Key features:
 * - 500Hz internal servo loop for smooth control
 * - Thread-safe command/state buffers
 * - Background send/recv mode support
 * - CAN interface management
 * - Health monitoring with tracking error calculation
 *
 * The adapter integrates with the external ARX SDK at runtime via dynamic
 * loading, maintaining the same protocol interface as the rest of rl_sar.
 */
class ArxAdapter
{
public:
    /**
     * @brief Configuration for the ARX adapter
     */
    struct Config
    {
        /// CAN interface name (e.g., "can0")
        std::string can_interface = "can0";

        /// External target update rate (Hz) - rate at which coordinator sends commands
        int target_rate_hz = 200;

        /// Internal servo rate (Hz) - rate of internal control loop
        int servo_rate_hz = 500;

        /// Enable background send/recv mode (ARX SDK feature)
        bool background_send_recv = true;

        /// Controller timestep (seconds) - matches servo_rate
        double controller_dt = 0.002;

        /// Require live state from backend on initialization
        bool require_live_state = true;

        /// Arm state timeout (milliseconds)
        double arm_state_timeout_ms = 50.0;

        /// Maximum allowed tracking error (radians)
        double arm_tracking_error_limit = 0.5;

        /// ARX SDK root path (or use ARX5_SDK_ROOT env var)
        std::string sdk_root;

        /// ARX SDK include directory
        std::string sdk_include_dir;

        /// ARX SDK library path
        std::string sdk_lib_path;

        /// Model type (e.g., "X5", "L5")
        std::string model = "X5";

        /// URDF path for the arm
        std::string urdf_path;

        /// Initialize to home position
        bool init_to_home = false;
    };

    /**
     * @brief Statistics for monitoring adapter health
     */
    struct Stats
    {
        uint64_t commands_sent = 0;
        uint64_t states_received = 0;
        uint64_t send_failures = 0;
        uint64_t state_timeouts = 0;
        uint64_t servo_loops = 0;
        double avg_send_latency_us = 0.0;
        double avg_state_latency_us = 0.0;
        double current_tracking_error = 0.0;
        bool backend_healthy = false;
    };

    /**
     * @brief Constructor
     */
    ArxAdapter();

    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~ArxAdapter();

    // Prevent copying
    ArxAdapter(const ArxAdapter&) = delete;
    ArxAdapter& operator=(const ArxAdapter&) = delete;

    /**
     * @brief Initialize the adapter with the given configuration
     *
     * @param config Configuration parameters
     * @return true if initialization successful
     */
    bool Initialize(const Config& config);

    /**
     * @brief Start the servo loop and begin communication
     *
     * Must be called after Initialize(). Starts the internal 500Hz servo thread.
     */
    void Start();

    /**
     * @brief Stop the servo loop and cleanup
     *
     * Safely stops the servo thread and releases resources.
     */
    void Stop();

    /**
     * @brief Set a new command for the arm
     *
     * Thread-safe. Stores the command for the servo loop to apply.
     *
     * @param cmd Command frame containing target positions, gains, etc.
     */
    void SetCommand(const protocol::ArmCommandFrame& cmd);

    /**
     * @brief Get the current arm state
     *
     * Thread-safe. Returns the most recent state from the backend.
     *
     * @param state Output parameter for current state
     * @return true if state is valid and recent
     */
    bool GetState(protocol::ArmStateFrame& state);

    /**
     * @brief Check if tracking is healthy
     *
     * Tracking is healthy if:
     * - State age is within timeout
     * - Tracking error is within limit
     * - Backend is responsive
     *
     * @return true if tracking is healthy
     */
    bool IsTrackingHealthy() const;

    /**
     * @brief Get the age of the current state in microseconds
     *
     * @return State age in microseconds
     */
    uint64_t GetStateAgeUs() const;

    /**
     * @brief Get the current tracking error
     *
     * Returns the L2 norm of (q_target - q).
     *
     * @return Tracking error in radians
     */
    double GetTrackingError() const;

    /**
     * @brief Get current statistics
     *
     * @return Current adapter statistics
     */
    Stats GetStats() const;

    /**
     * @brief Check if adapter is initialized
     *
     * @return true if Initialize() succeeded
     */
    bool IsInitialized() const { return initialized_; }

    /**
     * @brief Check if servo loop is running
     *
     * @return true if Start() was called and not stopped
     */
    bool IsRunning() const { return running_.load(); }

    /**
     * @brief Get the configuration used during initialization
     *
     * @return Current configuration
     */
    const Config& GetConfig() const { return config_; }

private:
    /**
     * @brief Internal state representation
     */
    struct InternalState
    {
        std::array<float, 6> q{};          ///< Current joint positions
        std::array<float, 6> dq{};         ///< Current joint velocities
        std::array<float, 6> tau{};        ///< Current joint torques
        std::array<float, 6> q_target{};   ///< Target joint positions
        std::array<float, 6> tracking_error{}; ///< Per-joint tracking errors
        uint64_t seq = 0;                  ///< State sequence number
        uint64_t target_seq_applied = 0;   ///< Last command seq applied
        std::chrono::steady_clock::time_point stamp; ///< Timestamp
        bool from_backend = false;         ///< True if from ARX backend

        InternalState() : stamp(std::chrono::steady_clock::now()) {}
    };

    /**
     * @brief Internal command representation
     */
    struct InternalCommand
    {
        std::array<float, 6> q{};      ///< Target positions
        std::array<float, 6> dq{};     ///< Target velocities
        std::array<float, 6> kp{};     ///< Position gains
        std::array<float, 6> kd{};     ///< Damping gains
        std::array<float, 6> tau{};    ///< Feedforward torques
        float gripper = 0.0f;          ///< Gripper target
        uint64_t seq = 0;              ///< Command sequence number
        uint64_t expire_ns = 0;        ///< Expiration timestamp
        bool valid = false;            ///< Command validity flag

        InternalCommand() = default;
    };

    // Initialization helpers
    bool LoadArxSdk(std::string* error);
    bool InitializeCanInterface(std::string* error);
    bool InitializeBackend(std::string* error);
    bool VerifyInitialState(std::string* error);
    void UnloadArxSdk();

    // Servo loop
    void ServoLoop();
    void SendCommand(const InternalCommand& cmd);
    bool ReceiveState(InternalState& state);
    void UpdateTrackingError();
    double ComputeTrackingErrorNorm() const;

    // Time utilities
    static uint64_t GetMonotonicNs();
    static uint64_t GetMonotonicUs();
    static bool IsCommandExpired(uint64_t now_ns, uint64_t expire_ns);

    // State management
    void StoreState(const InternalState& state);
    InternalCommand GetCommand() const;

    // Member variables
    Config config_;
    Stats stats_;

    // Thread management
    std::thread servo_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};

    // Thread-safe buffers
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    std::condition_variable command_cv_;

    InternalState current_state_;
    InternalCommand pending_command_;
    InternalCommand active_command_;

    // ARX SDK handle (opaque pointer for ABI stability)
    void* arx_sdk_handle_ = nullptr;

    // ARX backend interface pointers (function pointers from SDK)
    struct ArxVTable
    {
        bool (*create)(const char* model, const char* interface, int background_mode, double dt);
        void (*destroy)();
        bool (*send_command)(const float* q, const float* dq, const float* kp, const float* kd, const float* tau, int n);
        bool (*recv_state)(float* q, float* dq, float* tau, int n);
        bool (*set_gain)(const float* kp, const float* kd, int n);
        bool (*reset_to_home)();
        bool (*is_healthy)();
        void (*stop)();
    };
    ArxVTable arx_vtable_{};

    // Initialization state
    bool can_initialized_ = false;
    bool backend_initialized_ = false;
    bool require_live_state_failed_ = false;
};

}  // namespace rl_sar::adapters

#endif  // RL_SAR_ADAPTERS_ARX_ADAPTER_HPP
