#ifndef RL_SAR_ADAPTERS_ARX_BACKEND_INTERFACE_HPP
#define RL_SAR_ADAPTERS_ARX_BACKEND_INTERFACE_HPP

#include <cstdint>
#include <string>

#include "rl_sar/protocol/go2_x5_protocol_types.hpp"

namespace rl_sar::adapters::arx
{

class IArxBackend
{
public:
    virtual ~IArxBackend() = default;

    virtual bool Initialize(std::string* error) = 0;
    virtual void Stop() = 0;

    virtual bool SendCommand(const rl_sar::protocol::ArmCommandFrame& cmd, std::string* error) = 0;
    virtual bool ReceiveState(rl_sar::protocol::ArmStateFrame* state, std::string* error) = 0;

    virtual bool IsHealthy() const = 0;
    virtual std::string GetBackendName() const = 0;
    virtual uint64_t GetLastUpdateNs() const = 0;
};

}  // namespace rl_sar::adapters::arx

#endif  // RL_SAR_ADAPTERS_ARX_BACKEND_INTERFACE_HPP
