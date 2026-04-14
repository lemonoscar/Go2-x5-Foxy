#ifndef RL_SAR_DIAGNOSTICS_DIAGNOSTIC_AGGREGATOR_HPP
#define RL_SAR_DIAGNOSTICS_DIAGNOSTIC_AGGREGATOR_HPP

#include <mutex>
#include <string>

#include "rl_sar/diagnostics/diagnostic_types.hpp"

namespace rl_sar::diagnostics
{

class DiagnosticAggregator
{
public:
    void Aggregate(const SystemDiagnostics& diagnostics);
    std::string GenerateDetailedReport() const;
    std::string GenerateOneLineSummary() const;
    HealthStatus GetSystemHealth() const;
    bool HasError() const;
    bool HasWarning() const;
    SystemDiagnostics GetSnapshot() const;

private:
    SystemDiagnostics diagnostics_;
    mutable std::mutex mutex_;
};

}  // namespace rl_sar::diagnostics

#endif  // RL_SAR_DIAGNOSTICS_DIAGNOSTIC_AGGREGATOR_HPP
