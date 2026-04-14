#ifndef RL_SAR_DIAGNOSTICS_DIAGNOSTIC_TYPES_HPP
#define RL_SAR_DIAGNOSTICS_DIAGNOSTIC_TYPES_HPP

#include <cstdint>
#include <map>
#include <string>

namespace rl_sar::diagnostics
{

enum class HealthStatus
{
    OK = 0,
    Warning = 1,
    Error = 2,
    Failed = 3,
    Unknown = 4,
};

const char* ToString(HealthStatus health);

struct ComponentDiagnostics
{
    std::string component_name;
    std::string component_type;
    HealthStatus health = HealthStatus::Unknown;
    std::string message;
    uint64_t error_count = 0;
    uint64_t warning_count = 0;
    uint64_t last_error_time_ns = 0;
    std::map<std::string, std::string> details;
    uint64_t update_timestamp_ns = 0;
    uint64_t age_ns = 0;
};

struct SystemDiagnostics
{
    HealthStatus system_health = HealthStatus::Unknown;
    uint64_t total_errors = 0;
    uint64_t total_warnings = 0;
    std::map<std::string, ComponentDiagnostics> components;

    void AddComponent(const std::string& name, const ComponentDiagnostics& diag);
    ComponentDiagnostics GetComponent(const std::string& name) const;
    HealthStatus ComputeSystemHealth() const;
    std::string GenerateReport() const;
};

}  // namespace rl_sar::diagnostics

#endif  // RL_SAR_DIAGNOSTICS_DIAGNOSTIC_TYPES_HPP
