#ifndef GO2_X5_IPC_PROTOCOL_HPP
#define GO2_X5_IPC_PROTOCOL_HPP

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace Go2X5IPC
{

constexpr uint16_t kProtocolVersion = 1;
constexpr int kDefaultCommandPort = 45671;
constexpr int kDefaultStatePort = 45672;
constexpr int kDefaultJointCommandPort = 45673;
constexpr const char* kDefaultHost = "127.0.0.1";

inline std::string NormalizeTransport(std::string transport)
{
    std::transform(transport.begin(), transport.end(), transport.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (transport == "topic")
    {
        return "ros";
    }
    return transport;
}

inline bool IsIpcTransport(const std::string& transport)
{
    return NormalizeTransport(transport) == "ipc";
}

struct ArmCommandPacket
{
    uint16_t joint_count = 0;
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> kp;
    std::vector<float> kd;
    std::vector<float> tau;
};

struct ArmStatePacket
{
    uint16_t joint_count = 0;
    bool state_from_backend = false;
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
};

struct ArmPosePacket
{
    uint16_t joint_count = 0;
    std::vector<float> q;
};

inline size_t CommandPacketSize(const size_t joint_count)
{
    return 8 + joint_count * sizeof(float) * 5;
}

inline size_t StatePacketSize(const size_t joint_count)
{
    return 12 + joint_count * sizeof(float) * 3;
}

inline size_t PosePacketSize(const size_t joint_count)
{
    return 8 + joint_count * sizeof(float);
}

template <typename T>
inline void AppendScalar(std::vector<uint8_t>& bytes, const T& value)
{
    const auto* raw = reinterpret_cast<const uint8_t*>(&value);
    bytes.insert(bytes.end(), raw, raw + sizeof(T));
}

template <typename T>
inline bool ReadScalar(const std::vector<uint8_t>& bytes, size_t& offset, T& value)
{
    if (offset + sizeof(T) > bytes.size())
    {
        return false;
    }
    std::memcpy(&value, bytes.data() + offset, sizeof(T));
    offset += sizeof(T);
    return true;
}

inline std::vector<uint8_t> SerializeCommandPacket(
    const uint16_t joint_count,
    const std::vector<float>& q,
    const std::vector<float>& dq,
    const std::vector<float>& kp,
    const std::vector<float>& kd,
    const std::vector<float>& tau)
{
    std::vector<uint8_t> bytes;
    bytes.reserve(CommandPacketSize(joint_count));
    bytes.push_back('G');
    bytes.push_back('X');
    bytes.push_back('5');
    bytes.push_back('C');
    AppendScalar(bytes, kProtocolVersion);
    AppendScalar(bytes, joint_count);
    const auto append_vector = [&](const std::vector<float>& values)
    {
        for (uint16_t i = 0; i < joint_count; ++i)
        {
            const float value = i < values.size() ? values[static_cast<size_t>(i)] : 0.0f;
            AppendScalar(bytes, value);
        }
    };
    append_vector(q);
    append_vector(dq);
    append_vector(kp);
    append_vector(kd);
    append_vector(tau);
    return bytes;
}

inline std::vector<uint8_t> SerializeStatePacket(
    const uint16_t joint_count,
    const bool state_from_backend,
    const std::vector<float>& q,
    const std::vector<float>& dq,
    const std::vector<float>& tau)
{
    std::vector<uint8_t> bytes;
    bytes.reserve(StatePacketSize(joint_count));
    bytes.push_back('G');
    bytes.push_back('X');
    bytes.push_back('5');
    bytes.push_back('S');
    AppendScalar(bytes, kProtocolVersion);
    AppendScalar(bytes, joint_count);
    const uint32_t flags = state_from_backend ? 1u : 0u;
    AppendScalar(bytes, flags);
    const auto append_vector = [&](const std::vector<float>& values)
    {
        for (uint16_t i = 0; i < joint_count; ++i)
        {
            const float value = i < values.size() ? values[static_cast<size_t>(i)] : 0.0f;
            AppendScalar(bytes, value);
        }
    };
    append_vector(q);
    append_vector(dq);
    append_vector(tau);
    return bytes;
}

inline std::vector<uint8_t> SerializePosePacket(
    const uint16_t joint_count,
    const std::vector<float>& q)
{
    std::vector<uint8_t> bytes;
    bytes.reserve(PosePacketSize(joint_count));
    bytes.push_back('G');
    bytes.push_back('X');
    bytes.push_back('5');
    bytes.push_back('P');
    AppendScalar(bytes, kProtocolVersion);
    AppendScalar(bytes, joint_count);
    for (uint16_t i = 0; i < joint_count; ++i)
    {
        const float value = i < q.size() ? q[static_cast<size_t>(i)] : 0.0f;
        AppendScalar(bytes, value);
    }
    return bytes;
}

inline bool ParseCommandPacket(
    const std::vector<uint8_t>& bytes,
    ArmCommandPacket& packet,
    std::string* error = nullptr)
{
    if (bytes.size() < CommandPacketSize(0))
    {
        if (error) *error = "packet too short";
        return false;
    }
    if (!(bytes[0] == 'G' && bytes[1] == 'X' && bytes[2] == '5' && bytes[3] == 'C'))
    {
        if (error) *error = "invalid command packet magic";
        return false;
    }

    size_t offset = 4;
    uint16_t version = 0;
    uint16_t joint_count = 0;
    if (!ReadScalar(bytes, offset, version) || !ReadScalar(bytes, offset, joint_count))
    {
        if (error) *error = "failed to read command packet header";
        return false;
    }
    if (version != kProtocolVersion)
    {
        if (error) *error = "unsupported command packet version";
        return false;
    }
    if (bytes.size() != CommandPacketSize(joint_count))
    {
        if (error) *error = "unexpected command packet size";
        return false;
    }

    auto read_vector = [&](std::vector<float>& values) -> bool
    {
        values.assign(static_cast<size_t>(joint_count), 0.0f);
        for (uint16_t i = 0; i < joint_count; ++i)
        {
            if (!ReadScalar(bytes, offset, values[static_cast<size_t>(i)]))
            {
                return false;
            }
        }
        return true;
    };

    ArmCommandPacket local;
    local.joint_count = joint_count;
    if (!read_vector(local.q) || !read_vector(local.dq) || !read_vector(local.kp) ||
        !read_vector(local.kd) || !read_vector(local.tau))
    {
        if (error) *error = "failed to read command packet payload";
        return false;
    }
    packet = std::move(local);
    return true;
}

inline bool ParseStatePacket(
    const std::vector<uint8_t>& bytes,
    ArmStatePacket& packet,
    std::string* error = nullptr)
{
    if (bytes.size() < StatePacketSize(0))
    {
        if (error) *error = "packet too short";
        return false;
    }
    if (!(bytes[0] == 'G' && bytes[1] == 'X' && bytes[2] == '5' && bytes[3] == 'S'))
    {
        if (error) *error = "invalid state packet magic";
        return false;
    }

    size_t offset = 4;
    uint16_t version = 0;
    uint16_t joint_count = 0;
    uint32_t flags = 0;
    if (!ReadScalar(bytes, offset, version) || !ReadScalar(bytes, offset, joint_count) ||
        !ReadScalar(bytes, offset, flags))
    {
        if (error) *error = "failed to read state packet header";
        return false;
    }
    if (version != kProtocolVersion)
    {
        if (error) *error = "unsupported state packet version";
        return false;
    }
    if (bytes.size() != StatePacketSize(joint_count))
    {
        if (error) *error = "unexpected state packet size";
        return false;
    }

    auto read_vector = [&](std::vector<float>& values) -> bool
    {
        values.assign(static_cast<size_t>(joint_count), 0.0f);
        for (uint16_t i = 0; i < joint_count; ++i)
        {
            if (!ReadScalar(bytes, offset, values[static_cast<size_t>(i)]))
            {
                return false;
            }
        }
        return true;
    };

    ArmStatePacket local;
    local.joint_count = joint_count;
    local.state_from_backend = (flags & 1u) != 0u;
    if (!read_vector(local.q) || !read_vector(local.dq) || !read_vector(local.tau))
    {
        if (error) *error = "failed to read state packet payload";
        return false;
    }
    packet = std::move(local);
    return true;
}

inline bool ParsePosePacket(
    const std::vector<uint8_t>& bytes,
    ArmPosePacket& packet,
    std::string* error = nullptr)
{
    if (bytes.size() < PosePacketSize(0))
    {
        if (error) *error = "packet too short";
        return false;
    }
    if (!(bytes[0] == 'G' && bytes[1] == 'X' && bytes[2] == '5' && bytes[3] == 'P'))
    {
        if (error) *error = "invalid pose packet magic";
        return false;
    }

    size_t offset = 4;
    uint16_t version = 0;
    uint16_t joint_count = 0;
    if (!ReadScalar(bytes, offset, version) || !ReadScalar(bytes, offset, joint_count))
    {
        if (error) *error = "failed to read pose packet header";
        return false;
    }
    if (version != kProtocolVersion)
    {
        if (error) *error = "unsupported pose packet version";
        return false;
    }
    if (bytes.size() != PosePacketSize(joint_count))
    {
        if (error) *error = "unexpected pose packet size";
        return false;
    }

    ArmPosePacket local;
    local.joint_count = joint_count;
    local.q.assign(static_cast<size_t>(joint_count), 0.0f);
    for (uint16_t i = 0; i < joint_count; ++i)
    {
        if (!ReadScalar(bytes, offset, local.q[static_cast<size_t>(i)]))
        {
            if (error) *error = "failed to read pose packet payload";
            return false;
        }
    }
    packet = std::move(local);
    return true;
}

} // namespace Go2X5IPC

#endif // GO2_X5_IPC_PROTOCOL_HPP
