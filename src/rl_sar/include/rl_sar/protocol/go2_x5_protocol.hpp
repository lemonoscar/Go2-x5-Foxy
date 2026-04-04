#ifndef RL_SAR_PROTOCOL_GO2_X5_PROTOCOL_HPP
#define RL_SAR_PROTOCOL_GO2_X5_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

namespace rl_sar::protocol
{

std::vector<uint8_t> SerializeFrameHeader(const FrameHeader& header);
bool ParseFrameHeader(const std::vector<uint8_t>& bytes, FrameHeader& header, std::string* error = nullptr);

std::vector<uint8_t> SerializeBodyStateFrame(const BodyStateFrame& frame);
bool ParseBodyStateFrame(const std::vector<uint8_t>& bytes, BodyStateFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeArmStateFrame(const ArmStateFrame& frame);
bool ParseArmStateFrame(const std::vector<uint8_t>& bytes, ArmStateFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeOperatorCommandFrame(const OperatorCommandFrame& frame);
bool ParseOperatorCommandFrame(const std::vector<uint8_t>& bytes, OperatorCommandFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeDogPolicyObservationFrame(const DogPolicyObservationFrame& frame);
bool ParseDogPolicyObservationFrame(const std::vector<uint8_t>& bytes, DogPolicyObservationFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeDogPolicyCommandFrame(const DogPolicyCommandFrame& frame);
bool ParseDogPolicyCommandFrame(const std::vector<uint8_t>& bytes, DogPolicyCommandFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeArmCommandFrame(const ArmCommandFrame& frame);
bool ParseArmCommandFrame(const std::vector<uint8_t>& bytes, ArmCommandFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeBodyCommandFrame(const BodyCommandFrame& frame);
bool ParseBodyCommandFrame(const std::vector<uint8_t>& bytes, BodyCommandFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeHybridDiagnosticFrame(const HybridDiagnosticFrame& frame);
bool ParseHybridDiagnosticFrame(const std::vector<uint8_t>& bytes, HybridDiagnosticFrame& frame, std::string* error = nullptr);

std::vector<uint8_t> SerializeModeEventFrame(const ModeEventFrame& frame);
bool ParseModeEventFrame(const std::vector<uint8_t>& bytes, ModeEventFrame& frame, std::string* error = nullptr);

constexpr uint64_t AgeNs(uint64_t now_monotonic_ns, uint64_t source_monotonic_ns)
{
    return now_monotonic_ns > source_monotonic_ns ? (now_monotonic_ns - source_monotonic_ns) : 0u;
}

constexpr bool IsFresh(uint64_t now_monotonic_ns, uint64_t source_monotonic_ns, uint64_t max_age_ns)
{
    return AgeNs(now_monotonic_ns, source_monotonic_ns) <= max_age_ns;
}

inline bool IsFresh(const FrameHeader& header, uint64_t now_monotonic_ns, uint64_t max_age_ns)
{
    return IsFresh(now_monotonic_ns, header.source_monotonic_ns, max_age_ns);
}

constexpr uint64_t SeqGap(uint64_t previous_seq, uint64_t current_seq)
{
    return current_seq > previous_seq ? (current_seq - previous_seq) : 0u;
}

constexpr bool HasSeqGap(uint64_t previous_seq, uint64_t current_seq)
{
    return current_seq > (previous_seq + 1u);
}

inline bool HasSeqGap(const FrameHeader& previous, const FrameHeader& current)
{
    return HasSeqGap(previous.seq, current.seq);
}

constexpr bool IsCommandExpired(uint64_t now_monotonic_ns, uint64_t command_expire_ns)
{
    return command_expire_ns != 0u && now_monotonic_ns >= command_expire_ns;
}

}  // namespace rl_sar::protocol

#endif  // RL_SAR_PROTOCOL_GO2_X5_PROTOCOL_HPP
