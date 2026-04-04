#include "rl_sar/adapters/arx_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <errno.h>
#include <ifaddrs.h>
#include <linux/if.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rl_sar::adapters
{

namespace
{

constexpr uint16_t kArxJointCount = 6;
constexpr char kArxSdkLibName[] = "libarx5_sdk.so";
constexpr char kArxSdkLibNameAlt[] = "libarx5_interface.so";

// Logging utility - in production this would use the project's logger
void LogInfo(const std::string& msg)
{
    fprintf(stderr, "[ArxAdapter] [INFO] %s\n", msg.c_str());
}

void LogWarning(const std::string& msg)
{
    fprintf(stderr, "[ArxAdapter] [WARN] %s\n", msg.c_str());
}

void LogError(const std::string& msg)
{
    fprintf(stderr, "[ArxAdapter] [ERROR] %s\n", msg.c_str());
}

// Math utilities
template <typename T>
bool IsFinite(T value)
{
    return std::isfinite(value);
}

float L2Norm(const std::array<float, 6>& values)
{
    float sum = 0.0f;
    for (float v : values)
    {
        sum += v * v;
    }
    return std::sqrt(sum);
}

}  // namespace

// ============================================================================
// Public Interface
// ============================================================================

ArxAdapter::ArxAdapter() = default;

ArxAdapter::~ArxAdapter()
{
    Stop();
    UnloadArxSdk();
}

bool ArxAdapter::Initialize(const Config& config)
{
    if (initialized_.load())
    {
        LogWarning("Already initialized");
        return true;
    }

    config_ = config;
    std::string error;

    // Load ARX SDK
    if (!LoadArxSdk(&error))
    {
        LogError("Failed to load ARX SDK: " + error);
        if (config_.require_live_state)
        {
            return false;
        }
        LogWarning("Continuing without ARX SDK (shadow mode)");
    }

    // Initialize CAN interface
    if (!InitializeCanInterface(&error))
    {
        LogError("Failed to initialize CAN interface: " + error);
        return false;
    }

    // Initialize backend
    if (!InitializeBackend(&error))
    {
        LogError("Failed to initialize backend: " + error);
        if (config_.require_live_state)
        {
            return false;
        }
        LogWarning("Continuing without backend (shadow mode)");
    }
    else if (config_.require_live_state)
    {
        // Verify we have live state
        if (!VerifyInitialState(&error))
        {
            LogError("Initial state verification failed: " + error);
            require_live_state_failed_ = true;
            return false;
        }
    }

    // Initialize state
    current_state_.seq = 0;
    current_state_.stamp = std::chrono::steady_clock::now();

    // Reset stats
    stats_ = Stats{};
    stats_.backend_healthy = (backend_initialized_ && !require_live_state_failed_);

    initialized_.store(true);
    LogInfo("Initialized with model=" + config_.model +
            " interface=" + config_.can_interface +
            " servo_rate=" + std::to_string(config_.servo_rate_hz) + "Hz");

    return true;
}

void ArxAdapter::Start()
{
    if (!initialized_.load())
    {
        LogError("Cannot start: not initialized");
        return;
    }

    if (running_.load())
    {
        LogWarning("Already running");
        return;
    }

    running_.store(true);
    servo_thread_ = std::thread(&ArxAdapter::ServoLoop, this);

    LogInfo("Started servo loop at " + std::to_string(config_.servo_rate_hz) + "Hz");
}

void ArxAdapter::Stop()
{
    if (!running_.load())
    {
        return;
    }

    running_.store(false);
    command_cv_.notify_all();

    if (servo_thread_.joinable())
    {
        servo_thread_.join();
    }

    // Stop backend if available
    if (arx_vtable_.stop)
    {
        arx_vtable_.stop();
    }

    LogInfo("Stopped");
}

void ArxAdapter::SetCommand(const protocol::ArmCommandFrame& cmd)
{
    if (!initialized_.load())
    {
        LogWarning("SetCommand called before initialization");
        return;
    }

    InternalCommand internal;
    internal.seq = cmd.header.seq;
    internal.expire_ns = cmd.header.publish_monotonic_ns + cmd.command_expire_ns;
    internal.valid = true;

    // Copy joint data (ensure we only copy up to 6 joints)
    for (size_t i = 0; i < kArxJointCount && i < cmd.q.size(); ++i)
    {
        // Validate and clip values
        internal.q[i] = IsFinite(cmd.q[i]) ? cmd.q[i] : 0.0f;
        internal.dq[i] = IsFinite(cmd.dq[i]) ? cmd.dq[i] : 0.0f;
        internal.kp[i] = std::max(0.0f, IsFinite(cmd.kp[i]) ? cmd.kp[i] : 0.0f);
        internal.kd[i] = std::max(0.0f, IsFinite(cmd.kd[i]) ? cmd.kd[i] : 0.0f);
        internal.tau[i] = IsFinite(cmd.tau[i]) ? cmd.tau[i] : 0.0f;
    }
    internal.gripper = IsFinite(cmd.gripper_target) ? cmd.gripper_target : 0.0f;

    // Store command
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_ = internal;
    }
    command_cv_.notify_one();
}

bool ArxAdapter::GetState(protocol::ArmStateFrame& state)
{
    if (!initialized_.load())
    {
        return false;
    }

    InternalState internal;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        internal = current_state_;
    }

    // Check if state is fresh enough
    auto now = std::chrono::steady_clock::now();
    auto age_us = std::chrono::duration_cast<std::chrono::microseconds>(now - internal.stamp).count();
    double age_ms = static_cast<double>(age_us) / 1000.0;

    if (age_ms > config_.arm_state_timeout_ms)
    {
        ++stats_.state_timeouts;
        return false;
    }

    // Fill state frame
    state.header = internal.from_backend ?
        protocol::FrameHeader{} : protocol::FrameHeader{};

    state.header.source_monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        internal.stamp.time_since_epoch()).count();
    state.header.publish_monotonic_ns = GetMonotonicNs();
    state.header.seq = internal.seq;
    state.header.validity_flags = protocol::kValidityPayloadValid;

    if (internal.from_backend)
    {
        state.header.validity_flags |= protocol::kValidityFromBackend;
    }
    else
    {
        state.header.validity_flags |= protocol::kValidityShadowState;
    }

    state.joint_count = kArxJointCount;
    state.backend_mode = 0;
    state.q = internal.q;
    state.dq = internal.dq;
    state.tau = internal.tau;
    state.q_target = internal.q_target;
    state.tracking_error = internal.tracking_error;
    state.backend_age_us = internal.from_backend ? static_cast<uint32_t>(age_us) : 0;
    state.transport_age_us = 0;
    state.target_seq_applied = internal.target_seq_applied;

    return true;
}

bool ArxAdapter::IsTrackingHealthy() const
{
    if (!initialized_.load())
    {
        return false;
    }

    auto age_us = GetStateAgeUs();
    double age_ms = static_cast<double>(age_us) / 1000.0;

    if (age_ms > config_.arm_state_timeout_ms)
    {
        return false;
    }

    if (GetTrackingError() > config_.arm_tracking_error_limit)
    {
        return false;
    }

    if (!stats_.backend_healthy)
    {
        return false;
    }

    return true;
}

uint64_t ArxAdapter::GetStateAgeUs() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        now - current_state_.stamp).count());
}

double ArxAdapter::GetTrackingError() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return ComputeTrackingErrorNorm();
}

ArxAdapter::Stats ArxAdapter::GetStats() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    Stats stats = stats_;
    stats.current_tracking_error = ComputeTrackingErrorNorm();
    return stats;
}

// ============================================================================
// Initialization Helpers
// ============================================================================

bool ArxAdapter::LoadArxSdk(std::string* error)
{
    // Determine SDK library path
    std::string lib_path = config_.sdk_lib_path;

    if (lib_path.empty())
    {
        const char* env_root = std::getenv("ARX5_SDK_ROOT");
        if (env_root)
        {
            lib_path = std::string(env_root) + "/lib/libarx5_sdk.so";
        }
    }

    // Try to load the library
    arx_sdk_handle_ = dlopen(lib_path.c_str(), RTLD_LAZY);
    if (!arx_sdk_handle_ && !lib_path.empty())
    {
        *error = "dlopen failed: " + std::string(dlerror());
        return false;
    }

    // If no path specified, try common names
    if (!arx_sdk_handle_)
    {
        arx_sdk_handle_ = dlopen(kArxSdkLibName, RTLD_LAZY);
    }
    if (!arx_sdk_handle_)
    {
        arx_sdk_handle_ = dlopen(kArxSdkLibNameAlt, RTLD_LAZY);
    }
    if (!arx_sdk_handle_)
    {
        *error = "Could not find ARX SDK library (tried: " +
                 std::string(kArxSdkLibName) + ", " +
                 std::string(kArxSdkLibNameAlt) + ")";
        return false;
    }

    // Load function pointers
    auto load_sym = [this, error](const char* name, auto*& ptr) -> bool {
        ptr = reinterpret_cast<decltype(ptr)>(dlsym(arx_sdk_handle_, name));
        if (!ptr)
        {
            *error = "Missing symbol: " + std::string(name);
            return false;
        }
        return true;
    };

    // Note: These are example function names - actual ARX SDK may differ
    // In production, these would match the actual SDK interface
    // For now, we set dummy pointers to allow compilation
    arx_vtable_.create = nullptr;
    arx_vtable_.destroy = nullptr;
    arx_vtable_.send_command = nullptr;
    arx_vtable_.recv_state = nullptr;
    arx_vtable_.set_gain = nullptr;
    arx_vtable_.reset_to_home = nullptr;
    arx_vtable_.is_healthy = nullptr;
    arx_vtable_.stop = nullptr;

    LogInfo("ARX SDK loaded from: " + lib_path);
    return true;
}

void ArxAdapter::UnloadArxSdk()
{
    if (arx_vtable_.destroy)
    {
        arx_vtable_.destroy();
    }

    if (arx_sdk_handle_)
    {
        dlclose(arx_sdk_handle_);
        arx_sdk_handle_ = nullptr;
    }

    backend_initialized_ = false;
}

bool ArxAdapter::InitializeCanInterface(std::string* error)
{
    // Check if CAN interface exists
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        *error = "Failed to create socket: " + std::string(strerror(errno));
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, config_.can_interface.c_str(), IFNAMSIZ - 1);

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        close(sock);
        *error = "CAN interface '" + config_.can_interface + "' not found: " +
                 std::string(strerror(errno));
        return false;
    }

    close(sock);
    can_initialized_ = true;
    LogInfo("CAN interface '" + config_.can_interface + "' verified");
    return true;
}

bool ArxAdapter::InitializeBackend(std::string* error)
{
    if (!arx_sdk_handle_)
    {
        *error = "ARX SDK not loaded";
        return false;
    }

    if (!arx_vtable_.create)
    {
        *error = "ARX SDK vtable not initialized (SDK may not support direct C++ interface)";
        return false;
    }

    int background_mode = config_.background_send_recv ? 1 : 0;
    if (!arx_vtable_.create(config_.model.c_str(), config_.can_interface.c_str(),
                            background_mode, config_.controller_dt))
    {
        *error = "Backend creation failed";
        return false;
    }

    backend_initialized_ = true;

    // Reset to home if requested
    if (config_.init_to_home && arx_vtable_.reset_to_home)
    {
        arx_vtable_.reset_to_home();
        LogInfo("Reset arm to home position");
    }

    LogInfo("ARX backend initialized");
    return true;
}

bool ArxAdapter::VerifyInitialState(std::string* error)
{
    if (!backend_initialized_)
    {
        *error = "Backend not initialized";
        return false;
    }

    // Try to read state multiple times
    constexpr int kMaxRetries = 10;
    constexpr int kRetryDelayMs = 50;

    for (int i = 0; i < kMaxRetries; ++i)
    {
        InternalState state;
        if (ReceiveState(state))
        {
            // Check if state is non-zero (indicating live connection)
            bool has_nonzero = false;
            for (float v : state.q)
            {
                if (std::abs(v) > 1e-6f)
                {
                    has_nonzero = true;
                    break;
                }
            }

            if (has_nonzero)
            {
                StoreState(state);
                LogInfo("Verified initial state from backend");
                return true;
            }
        }

        usleep(kRetryDelayMs * 1000);
    }

    *error = "All-zero state after " + std::to_string(kMaxRetries) + " attempts";
    return false;
}

// ============================================================================
// Servo Loop
// ============================================================================

void ArxAdapter::ServoLoop()
{
    const std::chrono::microseconds servo_period{
        static_cast<int>(1000000 / config_.servo_rate_hz)};

    LogInfo("Servo loop started at " + std::to_string(config_.servo_rate_hz) + "Hz");

    while (running_.load())
    {
        auto loop_start = std::chrono::steady_clock::now();

        // Check for new command
        {
            std::unique_lock<std::mutex> lock(command_mutex_);
            if (pending_command_.valid)
            {
                active_command_ = pending_command_;
                pending_command_.valid = false;
            }
        }

        // Check if command is expired
        uint64_t now_ns = GetMonotonicNs();
        bool command_expired = IsCommandExpired(now_ns, active_command_.expire_ns);

        // Send command if not expired
        if (!command_expired && active_command_.valid)
        {
            auto send_start = std::chrono::steady_clock::now();
            SendCommand(active_command_);
            auto send_end = std::chrono::steady_clock::now();

            auto send_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                send_end - send_start).count();

            // Update avg latency (exponential moving average)
            const double alpha = 0.1;
            stats_.avg_send_latency_us = alpha * send_latency +
                (1.0 - alpha) * stats_.avg_send_latency_us;
        }

        // Receive state
        {
            auto recv_start = std::chrono::steady_clock::now();
            InternalState new_state;
            bool received = ReceiveState(new_state);
            auto recv_end = std::chrono::steady_clock::now();

            if (received)
            {
                auto recv_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                    recv_end - recv_start).count();

                const double alpha = 0.1;
                stats_.avg_state_latency_us = alpha * recv_latency +
                    (1.0 - alpha) * stats_.avg_state_latency_us;

                // Update tracking target from active command
                if (active_command_.valid)
                {
                    new_state.q_target = active_command_.q;
                    new_state.target_seq_applied = active_command_.seq;
                }

                // Update tracking error
                new_state.tracking_error = current_state_.tracking_error;
                for (size_t i = 0; i < kArxJointCount; ++i)
                {
                    new_state.tracking_error[i] = new_state.q_target[i] - new_state.q[i];
                }

                StoreState(new_state);
                ++stats_.states_received;
            }
        }

        // Update health status
        if (arx_vtable_.is_healthy)
        {
            stats_.backend_healthy = arx_vtable_.is_healthy();
        }

        ++stats_.servo_loops;

        // Sleep until next period
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            loop_end - loop_start);

        if (elapsed < servo_period)
        {
            std::this_thread::sleep_for(servo_period - elapsed);
        }
    }

    LogInfo("Servo loop terminated");
}

void ArxAdapter::SendCommand(const InternalCommand& cmd)
{
    if (!backend_initialized_ || !arx_vtable_.send_command)
    {
        ++stats_.send_failures;
        return;
    }

    bool success = arx_vtable_.send_command(
        cmd.q.data(), cmd.dq.data(), cmd.kp.data(),
        cmd.kd.data(), cmd.tau.data(), kArxJointCount);

    if (success)
    {
        ++stats_.commands_sent;
    }
    else
    {
        ++stats_.send_failures;
    }
}

bool ArxAdapter::ReceiveState(InternalState& state)
{
    if (!backend_initialized_ || !arx_vtable_.recv_state)
    {
        return false;
    }

    bool success = arx_vtable_.recv_state(
        state.q.data(), state.dq.data(), state.tau.data(), kArxJointCount);

    if (success)
    {
        state.from_backend = true;
        state.stamp = std::chrono::steady_clock::now();
    }

    return success;
}

void ArxAdapter::UpdateTrackingError()
{
    // Tracking error is updated when state is received
}

double ArxAdapter::ComputeTrackingErrorNorm() const
{
    return L2Norm(current_state_.tracking_error);
}

// ============================================================================
// State Management
// ============================================================================

void ArxAdapter::StoreState(const InternalState& state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_ = state;
    ++current_state_.seq;
}

ArxAdapter::InternalCommand ArxAdapter::GetCommand() const
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    return pending_command_;
}

// ============================================================================
// Time Utilities
// ============================================================================

uint64_t ArxAdapter::GetMonotonicNs()
{
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

uint64_t ArxAdapter::GetMonotonicUs()
{
    return GetMonotonicNs() / 1000;
}

bool ArxAdapter::IsCommandExpired(uint64_t now_ns, uint64_t expire_ns)
{
    return expire_ns != 0 && now_ns >= expire_ns;
}

}  // namespace rl_sar::adapters
