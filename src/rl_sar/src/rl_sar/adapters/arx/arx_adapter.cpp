#include "rl_sar/adapters/arx_adapter.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rl_sar/adapters/arx/arx_backend_interface.hpp"
#include "rl_sar/go2x5/comm/go2_x5_ipc_protocol.hpp"
#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

#if defined(__linux__)
#include <arpa/inet.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <linux/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace rl_sar::adapters
{

namespace
{

using rl_sar::protocol::ArmCommandFrame;
using rl_sar::protocol::ArmStateFrame;
using rl_sar::protocol::FrameType;

constexpr uint16_t kArxJointCount = rl_sar::protocol::kArmJointCount;
constexpr char kArxSdkHardwareLibName[] = "libhardware.so";

void LogInfo(const std::string& msg)
{
    std::fprintf(stderr, "[ArxAdapter] [INFO] %s\n", msg.c_str());
}

void LogWarning(const std::string& msg)
{
    std::fprintf(stderr, "[ArxAdapter] [WARN] %s\n", msg.c_str());
}

void LogError(const std::string& msg)
{
    std::fprintf(stderr, "[ArxAdapter] [ERROR] %s\n", msg.c_str());
}

template <typename T>
bool IsFinite(T value)
{
    return std::isfinite(value);
}

float L2Norm(const std::array<float, 6>& values)
{
    float sum = 0.0f;
    for (const float value : values)
    {
        sum += value * value;
    }
    return std::sqrt(sum);
}

uint64_t GetMonotonicNsNow()
{
    const auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

template <typename UInt>
void AppendUnsignedLE(std::vector<uint8_t>& bytes, UInt value)
{
    for (size_t i = 0; i < sizeof(UInt); ++i)
    {
        bytes.push_back(static_cast<uint8_t>((value >> (8u * i)) & 0xFFu));
    }
}

template <typename UInt>
bool ReadUnsignedLE(const std::vector<uint8_t>& bytes, size_t& offset, UInt& value)
{
    if (offset + sizeof(UInt) > bytes.size())
    {
        return false;
    }

    UInt result = 0;
    for (size_t i = 0; i < sizeof(UInt); ++i)
    {
        result |= static_cast<UInt>(bytes[offset + i]) << (8u * i);
    }
    offset += sizeof(UInt);
    value = result;
    return true;
}

template <typename T>
void AppendScalarLE(std::vector<uint8_t>& bytes, T value)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        using Raw = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
        Raw raw = 0;
        std::memcpy(&raw, &value, sizeof(T));
        AppendUnsignedLE(bytes, raw);
    }
    else if constexpr (std::is_enum_v<T>)
    {
        using Raw = std::underlying_type_t<T>;
        using Unsigned = std::make_unsigned_t<Raw>;
        AppendUnsignedLE(bytes, static_cast<Unsigned>(value));
    }
    else
    {
        using Unsigned = std::make_unsigned_t<T>;
        AppendUnsignedLE(bytes, static_cast<Unsigned>(value));
    }
}

template <typename T>
bool ReadScalarLE(const std::vector<uint8_t>& bytes, size_t& offset, T& value)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        using Raw = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
        Raw raw = 0;
        if (!ReadUnsignedLE(bytes, offset, raw))
        {
            return false;
        }
        std::memcpy(&value, &raw, sizeof(T));
        return true;
    }
    else if constexpr (std::is_enum_v<T>)
    {
        using Raw = std::underlying_type_t<T>;
        using Unsigned = std::make_unsigned_t<Raw>;
        Unsigned raw = 0;
        if (!ReadUnsignedLE(bytes, offset, raw))
        {
            return false;
        }
        value = static_cast<T>(raw);
        return true;
    }
    else
    {
        using Unsigned = std::make_unsigned_t<T>;
        Unsigned raw = 0;
        if (!ReadUnsignedLE(bytes, offset, raw))
        {
            return false;
        }
        value = static_cast<T>(raw);
        return true;
    }
}

template <typename T, size_t N>
void AppendArrayLE(std::vector<uint8_t>& bytes, const std::array<T, N>& values)
{
    for (const auto& value : values)
    {
        AppendScalarLE(bytes, value);
    }
}

template <typename T, size_t N>
bool ReadArrayLE(const std::vector<uint8_t>& bytes, size_t& offset, std::array<T, N>& values)
{
    for (auto& value : values)
    {
        if (!ReadScalarLE(bytes, offset, value))
        {
            return false;
        }
    }
    return true;
}

std::vector<uint8_t> SerializeTypedArmCommandFrame(const ArmCommandFrame& frame)
{
    ArmCommandFrame copy = frame;
    copy.header.magic = rl_sar::protocol::kFrameMagic;
    copy.header.version = rl_sar::protocol::kProtocolVersion;
    copy.header.msg_type = FrameType::ArmCommand;
    copy.header.payload_bytes = static_cast<uint32_t>(rl_sar::protocol::kArmCommandPayloadSize);
    copy.joint_count = kArxJointCount;

    std::vector<uint8_t> bytes;
    bytes.reserve(
        rl_sar::protocol::kFrameHeaderSize + rl_sar::protocol::kArmCommandPayloadSize);
    AppendScalarLE(bytes, copy.header.magic);
    AppendScalarLE(bytes, copy.header.version);
    AppendScalarLE(bytes, static_cast<uint16_t>(copy.header.msg_type));
    AppendScalarLE(bytes, copy.header.seq);
    AppendScalarLE(bytes, copy.header.source_monotonic_ns);
    AppendScalarLE(bytes, copy.header.publish_monotonic_ns);
    AppendScalarLE(bytes, copy.header.source_id);
    AppendScalarLE(bytes, copy.header.mode);
    AppendScalarLE(bytes, copy.header.validity_flags);
    AppendScalarLE(bytes, copy.header.fault_flags);
    AppendScalarLE(bytes, copy.header.payload_bytes);
    AppendScalarLE(bytes, copy.joint_count);
    AppendScalarLE(bytes, copy.interpolation_hint);
    AppendScalarLE(bytes, copy.command_expire_ns);
    AppendArrayLE(bytes, copy.q);
    AppendArrayLE(bytes, copy.dq);
    AppendArrayLE(bytes, copy.kp);
    AppendArrayLE(bytes, copy.kd);
    AppendArrayLE(bytes, copy.tau);
    AppendScalarLE(bytes, copy.gripper_target);
    return bytes;
}

bool ParseTypedArmStateFrame(
    const std::vector<uint8_t>& bytes,
    ArmStateFrame* frame,
    std::string* error)
{
    if (!frame)
    {
        if (error)
        {
            *error = "null frame";
        }
        return false;
    }
    const size_t expected_size =
        rl_sar::protocol::kFrameHeaderSize + rl_sar::protocol::kArmStatePayloadSize;
    if (bytes.size() != expected_size)
    {
        if (error)
        {
            *error = "unexpected typed frame size";
        }
        return false;
    }

    size_t offset = 0;
    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t msg_type = 0;
    if (!ReadScalarLE(bytes, offset, magic) ||
        !ReadScalarLE(bytes, offset, version) ||
        !ReadScalarLE(bytes, offset, msg_type) ||
        !ReadScalarLE(bytes, offset, frame->header.seq) ||
        !ReadScalarLE(bytes, offset, frame->header.source_monotonic_ns) ||
        !ReadScalarLE(bytes, offset, frame->header.publish_monotonic_ns) ||
        !ReadScalarLE(bytes, offset, frame->header.source_id) ||
        !ReadScalarLE(bytes, offset, frame->header.mode) ||
        !ReadScalarLE(bytes, offset, frame->header.validity_flags) ||
        !ReadScalarLE(bytes, offset, frame->header.fault_flags) ||
        !ReadScalarLE(bytes, offset, frame->header.payload_bytes))
    {
        if (error)
        {
            *error = "failed to parse typed arm state header";
        }
        return false;
    }

    frame->header.magic = magic;
    frame->header.version = version;
    frame->header.msg_type = static_cast<FrameType>(msg_type);
    if (magic != rl_sar::protocol::kFrameMagic)
    {
        if (error)
        {
            *error = "invalid typed arm state magic";
        }
        return false;
    }
    if (version != rl_sar::protocol::kProtocolVersion)
    {
        if (error)
        {
            *error = "unsupported typed arm state version";
        }
        return false;
    }
    if (frame->header.msg_type != FrameType::ArmState)
    {
        if (error)
        {
            *error = "unexpected typed frame type";
        }
        return false;
    }
    if (frame->header.payload_bytes != rl_sar::protocol::kArmStatePayloadSize)
    {
        if (error)
        {
            *error = "unexpected typed arm state payload";
        }
        return false;
    }

    if (!ReadScalarLE(bytes, offset, frame->joint_count) ||
        !ReadScalarLE(bytes, offset, frame->backend_mode) ||
        !ReadArrayLE(bytes, offset, frame->q) ||
        !ReadArrayLE(bytes, offset, frame->dq) ||
        !ReadArrayLE(bytes, offset, frame->tau) ||
        !ReadArrayLE(bytes, offset, frame->q_target) ||
        !ReadArrayLE(bytes, offset, frame->tracking_error) ||
        !ReadScalarLE(bytes, offset, frame->backend_age_us) ||
        !ReadScalarLE(bytes, offset, frame->transport_age_us) ||
        !ReadScalarLE(bytes, offset, frame->target_seq_applied))
    {
        if (error)
        {
            *error = "failed to parse typed arm state payload";
        }
        return false;
    }
    return true;
}

std::string DetectArxLibDirFromRoot(const std::string& root)
{
    if (root.empty())
    {
        return {};
    }

    namespace fs = std::filesystem;
    const fs::path base(root);
#if defined(__aarch64__)
    const fs::path arch_dir = base / "lib" / "aarch64";
#else
    const fs::path arch_dir = base / "lib" / "x86_64";
#endif
    if (fs::exists(arch_dir / kArxSdkHardwareLibName))
    {
        return arch_dir.string();
    }

    const fs::path flat_dir = base / "lib";
    if (fs::exists(flat_dir / kArxSdkHardwareLibName))
    {
        return flat_dir.string();
    }

    return {};
}

bool CheckInterfaceExists(const std::string& interface_name, std::string* error)
{
#if defined(__linux__)
    if (interface_name.empty())
    {
        if (error)
        {
            *error = "CAN interface is not configured";
        }
        return false;
    }

    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        if (error)
        {
            *error = "failed to create socket: " + std::string(std::strerror(errno));
        }
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    const int rc = ioctl(sock, SIOCGIFINDEX, &ifr);
    const std::string ioctl_error = std::strerror(errno);
    close(sock);
    if (rc < 0)
    {
        if (error)
        {
            *error = "CAN interface '" + interface_name + "' not found: " + ioctl_error;
        }
        return false;
    }
    return true;
#else
    (void)interface_name;
    if (error)
    {
        *error = "CAN interface probing is only supported on Linux";
    }
    return false;
#endif
}

bool MakeIpv4Endpoint(const std::string& host, const int port, sockaddr_in* addr)
{
    if (!addr || port <= 0 || port > 65535)
    {
        return false;
    }

    std::memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(static_cast<uint16_t>(port));
    if (host.empty())
    {
        addr->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        return true;
    }
    return inet_pton(AF_INET, host.c_str(), &addr->sin_addr) == 1;
}

enum class SdkMotorType : uint8_t
{
    EcA4310,
    DmJ4310,
    DmJ4340,
    DmJ8009,
    None,
};

struct SdkRobotConfig
{
    std::array<uint16_t, kArxJointCount> motor_id{};
    std::array<SdkMotorType, kArxJointCount> motor_type{};
    std::array<float, kArxJointCount> joint_pos_min{};
    std::array<float, kArxJointCount> joint_pos_max{};
    std::array<float, kArxJointCount> joint_vel_max{};
    std::array<float, kArxJointCount> joint_torque_max{};
    uint16_t gripper_motor_id = 8;
    SdkMotorType gripper_motor_type = SdkMotorType::DmJ4310;
    float gripper_width = 0.088f;
    float gripper_open_readout = 5.03f;
    float default_gripper_kp = 5.0f;
    float default_gripper_kd = 0.2f;
};

std::string NormalizeModelName(std::string model)
{
    std::transform(model.begin(), model.end(), model.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return model;
}

const SdkRobotConfig* ResolveSdkRobotConfig(const std::string& model)
{
    static const SdkRobotConfig kX5Config{
        {1, 2, 4, 5, 6, 7},
        {SdkMotorType::EcA4310, SdkMotorType::EcA4310, SdkMotorType::EcA4310,
         SdkMotorType::DmJ4310, SdkMotorType::DmJ4310, SdkMotorType::DmJ4310},
        {-3.14f, -0.05f, -0.1f, -1.6f, -1.57f, -2.0f},
        {2.618f, 3.50f, 3.20f, 1.55f, 1.57f, 2.0f},
        {5.0f, 5.0f, 5.5f, 5.5f, 5.0f, 5.0f},
        {30.0f, 40.0f, 30.0f, 15.0f, 10.0f, 10.0f},
        8,
        SdkMotorType::DmJ4310,
        0.088f,
        5.03f,
        5.0f,
        0.2f,
    };
    static const SdkRobotConfig kX5UmiConfig{
        {1, 2, 4, 5, 6, 7},
        {SdkMotorType::EcA4310, SdkMotorType::EcA4310, SdkMotorType::EcA4310,
         SdkMotorType::DmJ4310, SdkMotorType::DmJ4310, SdkMotorType::DmJ4310},
        {-3.14f, -0.05f, -0.1f, -1.6f, -1.57f, -2.0f},
        {2.618f, 3.50f, 3.20f, 1.55f, 1.57f, 2.0f},
        {5.0f, 5.0f, 5.5f, 5.5f, 5.0f, 5.0f},
        {30.0f, 40.0f, 30.0f, 15.0f, 10.0f, 10.0f},
        8,
        SdkMotorType::DmJ4310,
        0.086f,
        4.90f,
        5.0f,
        0.2f,
    };
    static const SdkRobotConfig kL5Config{
        {1, 2, 4, 5, 6, 7},
        {SdkMotorType::DmJ4340, SdkMotorType::DmJ4340, SdkMotorType::DmJ4340,
         SdkMotorType::DmJ4310, SdkMotorType::DmJ4310, SdkMotorType::DmJ4310},
        {-3.14f, -0.05f, -0.1f, -1.6f, -1.57f, -2.0f},
        {2.618f, 3.50f, 3.20f, 1.55f, 1.57f, 2.0f},
        {5.0f, 5.0f, 5.5f, 5.5f, 5.0f, 5.0f},
        {30.0f, 40.0f, 30.0f, 15.0f, 10.0f, 10.0f},
        8,
        SdkMotorType::DmJ4310,
        0.088f,
        5.03f,
        5.0f,
        0.2f,
    };
    static const SdkRobotConfig kL5UmiConfig{
        {1, 2, 4, 5, 6, 7},
        {SdkMotorType::DmJ4340, SdkMotorType::DmJ4340, SdkMotorType::DmJ4340,
         SdkMotorType::DmJ4310, SdkMotorType::DmJ4310, SdkMotorType::DmJ4310},
        {-3.14f, -0.05f, -0.1f, -1.6f, -1.57f, -2.0f},
        {2.618f, 3.50f, 3.20f, 1.55f, 1.57f, 2.0f},
        {5.0f, 5.0f, 5.5f, 5.5f, 5.0f, 5.0f},
        {30.0f, 40.0f, 30.0f, 15.0f, 10.0f, 10.0f},
        8,
        SdkMotorType::DmJ4310,
        0.086f,
        4.90f,
        5.0f,
        0.2f,
    };

    const std::string normalized = NormalizeModelName(model);
    if (normalized == "X5_UMI")
    {
        return &kX5UmiConfig;
    }
    if (normalized == "L5")
    {
        return &kL5Config;
    }
    if (normalized == "L5_UMI")
    {
        return &kL5UmiConfig;
    }
    if (normalized.empty() || normalized == "X5")
    {
        return &kX5Config;
    }
    return nullptr;
}

float TorqueConstantForMotor(const SdkMotorType motor_type)
{
    switch (motor_type)
    {
        case SdkMotorType::EcA4310:
            return 1.4f * 1.4f;
        case SdkMotorType::DmJ4310:
            return 0.424f;
        case SdkMotorType::DmJ4340:
            return 1.0f;
        case SdkMotorType::DmJ8009:
            return 1.0f;
        case SdkMotorType::None:
        default:
            return 1.0f;
    }
}

float ClampAbs(const float value, const float max_abs)
{
    if (!IsFinite(value) || max_abs <= 0.0f)
    {
        return 0.0f;
    }
    return std::clamp(value, -max_abs, max_abs);
}

std::string ResolveArxHardwareLibPath(const ArxAdapter::Config& config)
{
    namespace fs = std::filesystem;
    if (!config.sdk_lib_path.empty())
    {
        const fs::path sdk_path(config.sdk_lib_path);
        if (fs::is_regular_file(sdk_path))
        {
            return sdk_path.string();
        }
        if (fs::is_directory(sdk_path))
        {
            const fs::path candidate = sdk_path / kArxSdkHardwareLibName;
            if (fs::exists(candidate))
            {
                return candidate.string();
            }
        }
    }

    std::string root = config.sdk_root;
    if (root.empty())
    {
        if (const char* env_root = std::getenv("ARX5_SDK_ROOT"))
        {
            root = env_root;
        }
    }
    const std::string lib_dir = DetectArxLibDirFromRoot(root);
    if (lib_dir.empty())
    {
        return {};
    }
    return (fs::path(lib_dir) / kArxSdkHardwareLibName).string();
}

struct SdkOdMotorMsg
{
    int16_t current_actual_int = 0;
    float speed_actual_rad = 0.0f;
    float angle_actual_rad = 0.0f;
    uint16_t motor_id = 0;
    uint8_t temperature = 0;
    uint8_t error = 0;
    float angle_actual_float = 0.0f;
    float current_actual_float = 0.0f;
    float gripper_pos = 0.0f;
    float gripper_spd = 0.0f;
    float gripper_cur = 0.0f;
    float gripper_last_pos = 0.0f;
    float gripper_totalangle = 0.0f;
    float round_cnt = 0.0f;
};

class SdkArxCan
{
public:
    explicit SdkArxCan(std::string interface_name);
    ~SdkArxCan();

    void send_EC_motor_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    void query_EC_motor_pos(uint16_t motor_id);
    void query_EC_motor_vel(uint16_t motor_id);
    void query_EC_motor_current(uint16_t motor_id);
    void send_DM_motor_cmd(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    void enable_DM_motor(uint16_t motor_id);
    void reset_zero_readout(uint16_t motor_id);
    void clear(uint16_t motor_id);
    std::array<SdkOdMotorMsg, 10> get_motor_msg();
};

static_assert(sizeof(SdkArxCan) == 1, "SdkArxCan declaration must stay empty for dlopen backend");

bool HasLiveMotorSignal(const SdkOdMotorMsg& msg)
{
    return msg.motor_id != 0u || std::fabs(msg.angle_actual_rad) > 1e-6f ||
        std::fabs(msg.speed_actual_rad) > 1e-6f || std::fabs(msg.current_actual_float) > 1e-6f ||
        msg.temperature != 0u || msg.error != 0u;
}

template <typename Fn>
bool LoadRequiredSymbol(void* handle, const char* symbol_name, Fn* out, std::string* error)
{
#if defined(__linux__)
    dlerror();
    void* symbol = dlsym(handle, symbol_name);
    const char* dl_error = dlerror();
    if (dl_error != nullptr || symbol == nullptr)
    {
        if (error)
        {
            *error = std::string("missing ARX SDK symbol ") + symbol_name +
                (dl_error ? std::string(": ") + dl_error : std::string());
        }
        return false;
    }
    *out = reinterpret_cast<Fn>(symbol);
    return true;
#else
    (void)handle;
    (void)symbol_name;
    (void)out;
    if (error)
    {
        *error = "ARX SDK backend requires Linux dlopen";
    }
    return false;
#endif
}

class SdkBackend final : public rl_sar::adapters::arx::IArxBackend
{
public:
    explicit SdkBackend(ArxAdapter::Config config)
        : config_(std::move(config)),
          state_timeout_ns_(
              static_cast<uint64_t>(std::max(0.0, config_.arm_state_timeout_ms) * 1'000'000.0))
    {
    }

    ~SdkBackend() override
    {
        this->Stop();
    }

    bool Initialize(std::string* error) override
    {
#if defined(__linux__)
        namespace fs = std::filesystem;
        this->hardware_lib_path_ = ResolveArxHardwareLibPath(this->config_);
        if (this->hardware_lib_path_.empty())
        {
            if (error)
            {
                *error = "ARX SDK lib path not configured";
            }
            return false;
        }

        if (!fs::exists(this->hardware_lib_path_))
        {
            if (error)
            {
                *error = "ARX SDK hardware library not found: " + this->hardware_lib_path_;
            }
            return false;
        }

        std::string can_error;
        if (!CheckInterfaceExists(this->config_.can_interface, &can_error))
        {
            if (error)
            {
                *error = can_error;
            }
            return false;
        }

        this->robot_config_ = ResolveSdkRobotConfig(this->config_.model);
        if (this->robot_config_ == nullptr)
        {
            if (error)
            {
                *error = "unsupported ARX model for SDK backend: " + this->config_.model;
            }
            return false;
        }

        this->sdk_handle_ = dlopen(this->hardware_lib_path_.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (this->sdk_handle_ == nullptr)
        {
            if (error)
            {
                *error = "dlopen(" + this->hardware_lib_path_ + ") failed: " + dlerror();
            }
            return false;
        }

        std::string symbol_error;
        if (!this->LoadSymbols(&symbol_error))
        {
            this->Stop();
            if (error)
            {
                *error = symbol_error;
            }
            return false;
        }

        try
        {
            this->symbols_.ctor(this->can_instance(), this->config_.can_interface);
            this->can_constructed_ = true;
        }
        catch (const std::exception& ex)
        {
            this->Stop();
            if (error)
            {
                *error = "ARX SDK ArxCan ctor failed: " + std::string(ex.what());
            }
            return false;
        }
        catch (...)
        {
            this->Stop();
            if (error)
            {
                *error = "ARX SDK ArxCan ctor failed with unknown exception";
            }
            return false;
        }

        for (int i = 0; i < 3; ++i)
        {
            this->ProbeTransport();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        this->healthy_ = false;
        this->last_update_ns_ = 0;
        this->last_probe_ns_ = GetMonotonicNsNow();
        return true;
#else
        if (error)
        {
            *error = "ARX SDK backend requires Linux";
        }
        return false;
#endif
    }

    void Stop() override
    {
#if defined(__linux__)
        if (this->can_constructed_)
        {
            try
            {
                this->symbols_.dtor(this->can_instance());
            }
            catch (...)
            {
            }
            this->can_constructed_ = false;
        }
        if (this->sdk_handle_ != nullptr)
        {
            dlclose(this->sdk_handle_);
            this->sdk_handle_ = nullptr;
        }
#endif
        this->healthy_ = false;
        this->last_update_ns_ = 0;
        this->last_probe_ns_ = 0;
    }

    bool SendCommand(const ArmCommandFrame& cmd, std::string* error) override
    {
        if (!this->can_constructed_ || this->robot_config_ == nullptr)
        {
            if (error)
            {
                *error = "SDK backend is not initialized";
            }
            return false;
        }

        try
        {
            for (size_t i = 0; i < kArxJointCount; ++i)
            {
                const float q = std::clamp(
                    IsFinite(cmd.q[i]) ? cmd.q[i] : 0.0f,
                    this->robot_config_->joint_pos_min[i],
                    this->robot_config_->joint_pos_max[i]);
                const float dq = ClampAbs(
                    IsFinite(cmd.dq[i]) ? cmd.dq[i] : 0.0f,
                    this->robot_config_->joint_vel_max[i]);
                const float kp = std::clamp(IsFinite(cmd.kp[i]) ? cmd.kp[i] : 0.0f, 0.0f, 500.0f);
                const float kd = std::clamp(IsFinite(cmd.kd[i]) ? cmd.kd[i] : 0.0f, 0.0f, 5.0f);
                const float tau = ClampAbs(
                    IsFinite(cmd.tau[i]) ? cmd.tau[i] : 0.0f,
                    this->robot_config_->joint_torque_max[i]);
                const float tor_cmd = tau / TorqueConstantForMotor(this->robot_config_->motor_type[i]);
                const uint16_t motor_id = this->robot_config_->motor_id[i];

                switch (this->robot_config_->motor_type[i])
                {
                    case SdkMotorType::EcA4310:
                        this->symbols_.send_ec(this->can_instance(), motor_id, kp, kd, q, dq, tor_cmd);
                        break;
                    case SdkMotorType::DmJ4310:
                    case SdkMotorType::DmJ4340:
                    case SdkMotorType::DmJ8009:
                        this->symbols_.send_dm(this->can_instance(), motor_id, kp, kd, q, dq, tor_cmd);
                        break;
                    case SdkMotorType::None:
                    default:
                        break;
                }

                this->last_q_target_[i] = q;
            }

            if (this->robot_config_->gripper_motor_type != SdkMotorType::None &&
                this->robot_config_->gripper_width > 1e-6f)
            {
                const float gripper_pos = std::clamp(
                    IsFinite(cmd.gripper_target) ? cmd.gripper_target : 0.0f,
                    0.0f,
                    this->robot_config_->gripper_width);
                const float gripper_motor_pos = gripper_pos / this->robot_config_->gripper_width *
                    this->robot_config_->gripper_open_readout;
                this->symbols_.send_dm(
                    this->can_instance(),
                    this->robot_config_->gripper_motor_id,
                    this->robot_config_->default_gripper_kp,
                    this->robot_config_->default_gripper_kd,
                    gripper_motor_pos,
                    0.0f,
                    0.0f);
            }

            this->last_target_seq_ = cmd.header.seq;
            this->last_command_send_ns_ = GetMonotonicNsNow();
            this->healthy_ = true;
            return true;
        }
        catch (const std::exception& ex)
        {
            this->healthy_ = false;
            if (error)
            {
                *error = "ARX SDK send failed: " + std::string(ex.what());
            }
            return false;
        }
        catch (...)
        {
            this->healthy_ = false;
            if (error)
            {
                *error = "ARX SDK send failed with unknown exception";
            }
            return false;
        }
    }

    bool ReceiveState(ArmStateFrame* state, std::string* error) override
    {
        if (!state)
        {
            if (error)
            {
                *error = "SDK backend state output is null";
            }
            return false;
        }
        if (!this->can_constructed_ || this->robot_config_ == nullptr)
        {
            if (error)
            {
                *error = "SDK backend is not initialized";
            }
            return false;
        }

        const uint64_t now_ns = GetMonotonicNsNow();
        if (this->last_command_send_ns_ == 0u ||
            now_ns - this->last_probe_ns_ > 20'000'000ULL)
        {
            this->ProbeTransport();
            this->last_probe_ns_ = now_ns;
        }

        std::array<SdkOdMotorMsg, 10> motor_msg{};
        try
        {
            motor_msg = this->symbols_.get_motor_msg(this->can_instance());
        }
        catch (const std::exception& ex)
        {
            this->healthy_ = false;
            if (error)
            {
                *error = "ARX SDK get_motor_msg failed: " + std::string(ex.what());
            }
            return false;
        }
        catch (...)
        {
            this->healthy_ = false;
            if (error)
            {
                *error = "ARX SDK get_motor_msg failed with unknown exception";
            }
            return false;
        }

        bool has_live_signal = false;
        *state = ArmStateFrame{};
        state->backend_mode = static_cast<uint16_t>(ArxAdapter::BackendType::InProcessSdk);
        state->header.validity_flags =
            rl_sar::protocol::kValidityPayloadValid | rl_sar::protocol::kValidityFromBackend;
        state->joint_count = kArxJointCount;
        state->target_seq_applied = this->last_target_seq_;
        state->q_target = this->last_q_target_;

        for (size_t i = 0; i < kArxJointCount; ++i)
        {
            const uint16_t motor_id = this->robot_config_->motor_id[i];
            if (motor_id >= motor_msg.size())
            {
                continue;
            }
            const auto& msg = motor_msg[motor_id];
            has_live_signal = has_live_signal || HasLiveMotorSignal(msg);
            state->q[i] = IsFinite(msg.angle_actual_rad) ? msg.angle_actual_rad : 0.0f;
            state->dq[i] = IsFinite(msg.speed_actual_rad) ? msg.speed_actual_rad : 0.0f;
            state->tau[i] =
                (IsFinite(msg.current_actual_float) ? msg.current_actual_float : 0.0f) *
                TorqueConstantForMotor(this->robot_config_->motor_type[i]);
            state->tracking_error[i] = state->q_target[i] - state->q[i];
        }

        const auto& gripper_msg = motor_msg[this->robot_config_->gripper_motor_id];
        has_live_signal = has_live_signal || HasLiveMotorSignal(gripper_msg);
        if (!has_live_signal)
        {
            return false;
        }

        state->header.seq = ++this->state_seq_;
        state->header.source_monotonic_ns = now_ns;
        state->header.publish_monotonic_ns = now_ns;
        state->backend_age_us = 0;
        state->transport_age_us = 0;
        this->last_update_ns_ = now_ns;
        this->healthy_ = true;
        return true;
    }

    bool IsHealthy() const override
    {
        if (!this->healthy_ || this->last_update_ns_ == 0u)
        {
            return false;
        }
        return GetMonotonicNsNow() - this->last_update_ns_ <= this->state_timeout_ns_;
    }

    std::string GetBackendName() const override
    {
        return "sdk_inprocess_arxcan";
    }

    uint64_t GetLastUpdateNs() const override
    {
        return this->last_update_ns_;
    }

private:
    struct Symbols
    {
        using Ctor = void (*)(SdkArxCan*, std::string);
        using Dtor = void (*)(SdkArxCan*);
        using SendEc = void (*)(SdkArxCan*, uint16_t, float, float, float, float, float);
        using QueryEc = void (*)(SdkArxCan*, uint16_t);
        using SendDm = void (*)(SdkArxCan*, uint16_t, float, float, float, float, float);
        using EnableDm = void (*)(SdkArxCan*, uint16_t);
        using ResetZeroReadout = void (*)(SdkArxCan*, uint16_t);
        using Clear = void (*)(SdkArxCan*, uint16_t);
        using GetMotorMsg = std::array<SdkOdMotorMsg, 10> (*)(SdkArxCan*);

        Ctor ctor = nullptr;
        Dtor dtor = nullptr;
        SendEc send_ec = nullptr;
        QueryEc query_ec_pos = nullptr;
        QueryEc query_ec_vel = nullptr;
        QueryEc query_ec_current = nullptr;
        SendDm send_dm = nullptr;
        EnableDm enable_dm = nullptr;
        ResetZeroReadout reset_zero_readout = nullptr;
        Clear clear = nullptr;
        GetMotorMsg get_motor_msg = nullptr;
    };

    bool LoadSymbols(std::string* error)
    {
        return LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCanC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE", &this->symbols_.ctor, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCanD1Ev", &this->symbols_.dtor, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan17send_EC_motor_cmdEtfffff", &this->symbols_.send_ec, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan18query_EC_motor_posEt", &this->symbols_.query_ec_pos, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan18query_EC_motor_velEt", &this->symbols_.query_ec_vel, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan22query_EC_motor_currentEt", &this->symbols_.query_ec_current, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan17send_DM_motor_cmdEtfffff", &this->symbols_.send_dm, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan15enable_DM_motorEt", &this->symbols_.enable_dm, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan18reset_zero_readoutEt", &this->symbols_.reset_zero_readout, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan5clearEt", &this->symbols_.clear, error) &&
            LoadRequiredSymbol(this->sdk_handle_, "_ZN6ArxCan13get_motor_msgEv", &this->symbols_.get_motor_msg, error);
    }

    SdkArxCan* can_instance()
    {
        return reinterpret_cast<SdkArxCan*>(&this->can_storage_);
    }

    const SdkArxCan* can_instance() const
    {
        return reinterpret_cast<const SdkArxCan*>(&this->can_storage_);
    }

    void ProbeTransport()
    {
        if (!this->can_constructed_ || this->robot_config_ == nullptr)
        {
            return;
        }

        for (size_t i = 0; i < kArxJointCount; ++i)
        {
            const uint16_t motor_id = this->robot_config_->motor_id[i];
            switch (this->robot_config_->motor_type[i])
            {
                case SdkMotorType::EcA4310:
                    this->symbols_.query_ec_pos(this->can_instance(), motor_id);
                    this->symbols_.query_ec_vel(this->can_instance(), motor_id);
                    this->symbols_.query_ec_current(this->can_instance(), motor_id);
                    break;
                case SdkMotorType::DmJ4310:
                case SdkMotorType::DmJ4340:
                case SdkMotorType::DmJ8009:
                    this->symbols_.enable_dm(this->can_instance(), motor_id);
                    break;
                case SdkMotorType::None:
                default:
                    break;
            }
        }

        if (this->robot_config_->gripper_motor_type != SdkMotorType::None)
        {
            this->symbols_.enable_dm(this->can_instance(), this->robot_config_->gripper_motor_id);
        }
    }

    ArxAdapter::Config config_;
    const SdkRobotConfig* robot_config_ = nullptr;
    std::string hardware_lib_path_;
    uint64_t state_timeout_ns_ = 0;
    bool healthy_ = false;
    uint64_t last_update_ns_ = 0;
    uint64_t last_probe_ns_ = 0;
    uint64_t last_command_send_ns_ = 0;
    uint64_t last_target_seq_ = 0;
    uint64_t state_seq_ = 0;
    std::array<float, kArxJointCount> last_q_target_{};
    std::aligned_storage_t<sizeof(SdkArxCan), alignof(SdkArxCan)> can_storage_{};
    void* sdk_handle_ = nullptr;
    bool can_constructed_ = false;
    Symbols symbols_{};
};

// BridgeBackend removed - only InProcessSdk is supported
// Bridge IPC functionality has been removed for simpler, more reliable deployment

}  // namespace

ArxAdapter::ArxAdapter() = default;

ArxAdapter::~ArxAdapter()
{
    Stop();
}

bool ArxAdapter::Initialize(const Config& config)
{
    if (initialized_.load())
    {
        LogWarning("Already initialized");
        return true;
    }

    config_ = config;
    stats_ = Stats{};
    current_state_ = InternalState{};
    pending_command_ = InternalCommand{};
    active_command_ = InternalCommand{};
    backend_.reset();
    backend_initialized_ = false;
    require_live_state_failed_ = false;
    active_backend_ = BackendType::None;

    std::string error;
    if (!InitializeBackend(&error))
    {
        if (config_.require_live_state)
        {
            LogError("Failed to initialize backend: " + error);
            return false;
        }
        LogWarning("Proceeding without active backend: " + error);
    }
    else if (config_.require_live_state)
    {
        if (!VerifyInitialState(&error))
        {
            LogError("Initial state verification failed: " + error);
            require_live_state_failed_ = true;
            return false;
        }
    }

    current_state_.stamp = std::chrono::steady_clock::now();
    stats_.backend_healthy = backend_ && backend_->IsHealthy() && !require_live_state_failed_;
    stats_.active_backend = active_backend_;
    stats_.backend_name = GetBackendName();
    stats_.backend_age_us = 0;
    initialized_.store(true);

    LogInfo(
        "Initialized backend=" + GetBackendName() +
        " can=" + config_.can_interface +
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
    if (running_.load())
    {
        running_.store(false);
        command_cv_.notify_all();
        if (servo_thread_.joinable())
        {
            servo_thread_.join();
        }
    }

    if (backend_)
    {
        backend_->Stop();
    }
    backend_initialized_ = false;
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
    internal.expire_ns = cmd.command_expire_ns;
    internal.valid = true;
    for (size_t i = 0; i < kArxJointCount && i < cmd.q.size(); ++i)
    {
        internal.q[i] = IsFinite(cmd.q[i]) ? cmd.q[i] : 0.0f;
        internal.dq[i] = IsFinite(cmd.dq[i]) ? cmd.dq[i] : 0.0f;
        internal.kp[i] = std::max(0.0f, IsFinite(cmd.kp[i]) ? cmd.kp[i] : 0.0f);
        internal.kd[i] = std::max(0.0f, IsFinite(cmd.kd[i]) ? cmd.kd[i] : 0.0f);
        internal.tau[i] = IsFinite(cmd.tau[i]) ? cmd.tau[i] : 0.0f;
    }
    internal.gripper = IsFinite(cmd.gripper_target) ? cmd.gripper_target : 0.0f;

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

    const auto now = std::chrono::steady_clock::now();
    const auto age_us = std::chrono::duration_cast<std::chrono::microseconds>(
        now - internal.stamp).count();
    const double age_ms = static_cast<double>(age_us) / 1000.0;
    if (age_ms > config_.arm_state_timeout_ms)
    {
        ++stats_.state_timeouts;
        return false;
    }

    state = protocol::ArmStateFrame{};
    state.header.msg_type = FrameType::ArmState;
    state.header.source_monotonic_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            internal.stamp.time_since_epoch()).count());
    state.header.publish_monotonic_ns = GetMonotonicNs();
    state.header.seq = internal.seq;
    state.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
    if (internal.from_backend)
    {
        state.header.validity_flags |= rl_sar::protocol::kValidityFromBackend;
    }
    else
    {
        state.header.validity_flags |= rl_sar::protocol::kValidityShadowState;
    }
    state.joint_count = kArxJointCount;
    state.backend_mode = static_cast<uint16_t>(active_backend_);
    state.q = internal.q;
    state.dq = internal.dq;
    state.tau = internal.tau;
    state.q_target = internal.q_target;
    state.tracking_error = internal.tracking_error;
    state.backend_age_us = internal.from_backend ? static_cast<uint32_t>(age_us) : 0u;
    state.transport_age_us = 0u;
    state.target_seq_applied = internal.target_seq_applied;
    return true;
}

bool ArxAdapter::IsTrackingHealthy() const
{
    if (!initialized_.load())
    {
        return false;
    }
    const double age_ms = static_cast<double>(GetStateAgeUs()) / 1000.0;
    if (age_ms > config_.arm_state_timeout_ms)
    {
        return false;
    }
    if (GetTrackingError() > config_.arm_tracking_error_limit)
    {
        return false;
    }
    return stats_.backend_healthy;
}

uint64_t ArxAdapter::GetStateAgeUs() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_state_.stamp.time_since_epoch().count() == 0)
    {
        return std::numeric_limits<uint64_t>::max();
    }
    const auto now = std::chrono::steady_clock::now();
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
    stats.active_backend = active_backend_;
    stats.backend_name = GetBackendName();
    if (backend_ && backend_->GetLastUpdateNs() > 0)
    {
        stats.backend_age_us = (GetMonotonicNs() - backend_->GetLastUpdateNs()) / 1000ULL;
        stats.backend_healthy = backend_->IsHealthy();
    }
    else
    {
        stats.backend_age_us = 0;
        stats.backend_healthy = backend_ && backend_->IsHealthy();
    }
    return stats;
}

ArxAdapter::BackendType ArxAdapter::GetActiveBackend() const
{
    return active_backend_;
}

std::string ArxAdapter::GetBackendName() const
{
    return backend_ ? backend_->GetBackendName() : "none";
}

bool ArxAdapter::InitializeBackend(std::string* error)
{
    backend_initialized_ = false;
    active_backend_ = BackendType::None;
    backend_.reset();

    // Only InProcessSdk is supported - BridgeBackend has been removed
    auto sdk_backend = std::make_unique<SdkBackend>(config_);
    if (!sdk_backend)
    {
        if (error)
        {
            *error = "SDK backend factory returned null";
        }
        return false;
    }

    if (!sdk_backend->Initialize(error))
    {
        return false;
    }

    backend_ = std::move(sdk_backend);
    active_backend_ = BackendType::InProcessSdk;
    backend_initialized_ = true;
    return true;
}

bool ArxAdapter::VerifyInitialState(std::string* error)
{
    if (!backend_initialized_)
    {
        if (error)
        {
            *error = "backend not initialized";
        }
        return false;
    }

    constexpr int kMaxRetries = 10;
    constexpr int kRetryDelayMs = 50;
    for (int attempt = 0; attempt < kMaxRetries; ++attempt)
    {
        InternalState state;
        if (ReceiveState(state))
        {
            bool has_nonzero = state.from_backend;
            for (const float value : state.q)
            {
                if (std::fabs(value) > 1e-6f)
                {
                    has_nonzero = true;
                    break;
                }
            }
            if (has_nonzero || !config_.require_live_state)
            {
                StoreState(state);
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kRetryDelayMs));
    }

    if (error)
    {
        *error = "no live arm state received after bridge/backend probe";
    }
    return false;
}

void ArxAdapter::ServoLoop()
{
    const std::chrono::microseconds servo_period{
        static_cast<int>(1000000 / std::max(1, config_.servo_rate_hz))};

    while (running_.load())
    {
        const auto loop_start = std::chrono::steady_clock::now();

        {
            std::unique_lock<std::mutex> lock(command_mutex_);
            if (pending_command_.valid)
            {
                active_command_ = pending_command_;
                pending_command_.valid = false;
            }
        }

        const uint64_t now_ns = GetMonotonicNs();
        const bool command_expired = IsCommandExpired(now_ns, active_command_.expire_ns);
        if (!command_expired && active_command_.valid)
        {
            const auto send_start = std::chrono::steady_clock::now();
            SendCommand(active_command_);
            const auto send_end = std::chrono::steady_clock::now();
            const auto send_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                send_end - send_start).count();
            constexpr double kAlpha = 0.1;
            stats_.avg_send_latency_us =
                kAlpha * static_cast<double>(send_latency) +
                (1.0 - kAlpha) * stats_.avg_send_latency_us;
        }

        {
            const auto recv_start = std::chrono::steady_clock::now();
            InternalState new_state;
            const bool received = ReceiveState(new_state);
            const auto recv_end = std::chrono::steady_clock::now();
            if (received)
            {
                const auto recv_latency = std::chrono::duration_cast<std::chrono::microseconds>(
                    recv_end - recv_start).count();
                constexpr double kAlpha = 0.1;
                stats_.avg_state_latency_us =
                    kAlpha * static_cast<double>(recv_latency) +
                    (1.0 - kAlpha) * stats_.avg_state_latency_us;

                if (active_command_.valid)
                {
                    new_state.q_target = active_command_.q;
                    new_state.target_seq_applied = active_command_.seq;
                }
                for (size_t i = 0; i < kArxJointCount; ++i)
                {
                    new_state.tracking_error[i] = new_state.q_target[i] - new_state.q[i];
                }
                StoreState(new_state);
                ++stats_.states_received;
            }
        }

        stats_.backend_healthy = backend_ && backend_->IsHealthy();
        stats_.active_backend = active_backend_;
        stats_.backend_name = GetBackendName();
        if (backend_ && backend_->GetLastUpdateNs() > 0)
        {
            stats_.backend_age_us = (GetMonotonicNs() - backend_->GetLastUpdateNs()) / 1000ULL;
        }
        ++stats_.servo_loops;

        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start);
        if (elapsed < servo_period)
        {
            std::this_thread::sleep_for(servo_period - elapsed);
        }
    }
}

void ArxAdapter::SendCommand(const InternalCommand& cmd)
{
    if (!backend_initialized_ || !backend_)
    {
        return;
    }

    ArmCommandFrame frame;
    frame.header.msg_type = FrameType::ArmCommand;
    frame.header.seq = cmd.seq;
    frame.header.source_monotonic_ns = GetMonotonicNs();
    frame.header.publish_monotonic_ns = frame.header.source_monotonic_ns;
    frame.header.validity_flags = rl_sar::protocol::kValidityPayloadValid;
    frame.joint_count = kArxJointCount;
    frame.command_expire_ns = cmd.expire_ns;
    frame.q = cmd.q;
    frame.dq = cmd.dq;
    frame.kp = cmd.kp;
    frame.kd = cmd.kd;
    frame.tau = cmd.tau;
    frame.gripper_target = cmd.gripper;

    std::string error;
    if (!backend_->SendCommand(frame, &error))
    {
        ++stats_.send_failures;
        if (!error.empty())
        {
            LogWarning(error);
        }
        return;
    }
    ++stats_.commands_sent;
}

bool ArxAdapter::ReceiveState(InternalState& state)
{
    if (!backend_initialized_ || !backend_)
    {
        return false;
    }

    ArmStateFrame frame;
    std::string error;
    if (!backend_->ReceiveState(&frame, &error))
    {
        return false;
    }

    state = InternalState{};
    state.stamp = std::chrono::steady_clock::now();
    state.from_backend =
        (frame.header.validity_flags & rl_sar::protocol::kValidityFromBackend) != 0u;
    state.target_seq_applied = frame.target_seq_applied;
    state.q = frame.q;
    state.dq = frame.dq;
    state.tau = frame.tau;
    state.q_target = frame.q_target;
    state.tracking_error = frame.tracking_error;
    return true;
}

void ArxAdapter::UpdateTrackingError()
{
}

double ArxAdapter::ComputeTrackingErrorNorm() const
{
    return L2Norm(current_state_.tracking_error);
}

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

uint64_t ArxAdapter::GetMonotonicNs()
{
    return GetMonotonicNsNow();
}

uint64_t ArxAdapter::GetMonotonicUs()
{
    return GetMonotonicNs() / 1000ULL;
}

bool ArxAdapter::IsCommandExpired(uint64_t now_ns, uint64_t expire_ns)
{
    return expire_ns != 0u && now_ns >= expire_ns;
}

}  // namespace rl_sar::adapters
