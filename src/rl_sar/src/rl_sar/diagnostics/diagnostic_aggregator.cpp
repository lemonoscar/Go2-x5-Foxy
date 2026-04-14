#include "rl_sar/diagnostics/diagnostic_aggregator.hpp"

#include <sstream>

namespace rl_sar::diagnostics
{

namespace
{

int SeverityRank(const HealthStatus health)
{
    switch (health)
    {
        case HealthStatus::OK: return 0;
        case HealthStatus::Warning: return 1;
        case HealthStatus::Error: return 2;
        case HealthStatus::Failed: return 3;
        case HealthStatus::Unknown: return 4;
    }
    return 4;
}

const char* HealthIcon(const HealthStatus health)
{
    switch (health)
    {
        case HealthStatus::OK: return "[OK]";
        case HealthStatus::Warning: return "[!]";
        case HealthStatus::Error: return "[ERR]";
        case HealthStatus::Failed: return "[FAIL]";
        case HealthStatus::Unknown: return "[?]";
    }
    return "[?]";
}

std::string ComponentSummary(const ComponentDiagnostics& component)
{
    std::ostringstream oss;
    oss << HealthIcon(component.health) << ' ' << component.component_name;
    if (!component.message.empty() && component.message != "OK")
    {
        oss << ": " << component.message;
    }
    return oss.str();
}

}  // namespace

const char* ToString(const HealthStatus health)
{
    switch (health)
    {
        case HealthStatus::OK: return "OK";
        case HealthStatus::Warning: return "Warning";
        case HealthStatus::Error: return "Error";
        case HealthStatus::Failed: return "Failed";
        case HealthStatus::Unknown: return "Unknown";
    }
    return "Unknown";
}

void SystemDiagnostics::AddComponent(const std::string& name, const ComponentDiagnostics& diag)
{
    this->components[name] = diag;
}

ComponentDiagnostics SystemDiagnostics::GetComponent(const std::string& name) const
{
    const auto it = this->components.find(name);
    if (it == this->components.end())
    {
        ComponentDiagnostics fallback;
        fallback.component_name = name;
        fallback.health = HealthStatus::Unknown;
        fallback.message = "missing";
        return fallback;
    }
    return it->second;
}

HealthStatus SystemDiagnostics::ComputeSystemHealth() const
{
    HealthStatus worst = HealthStatus::OK;
    for (const auto& entry : this->components)
    {
        if (SeverityRank(entry.second.health) > SeverityRank(worst))
        {
            worst = entry.second.health;
        }
    }
    return this->components.empty() ? HealthStatus::Unknown : worst;
}

std::string SystemDiagnostics::GenerateReport() const
{
    std::ostringstream oss;
    oss << "===== System Diagnostics =====\n";
    oss << "System Health: " << ToString(this->ComputeSystemHealth())
        << " (errors=" << this->total_errors
        << ", warnings=" << this->total_warnings << ")\n";

    for (const auto& entry : this->components)
    {
        const auto& component = entry.second;
        oss << '\n' << ComponentSummary(component) << '\n';
        oss << "  Type: " << component.component_type << '\n';
        if (!component.message.empty())
        {
            oss << "  Message: " << component.message << '\n';
        }
        oss << "  Age: " << component.age_ns / 1000000ULL << "ms\n";
        if (!component.details.empty())
        {
            oss << "  Details:";
            bool first = true;
            for (const auto& detail : component.details)
            {
                oss << (first ? " " : ", ") << detail.first << '=' << detail.second;
                first = false;
            }
            oss << '\n';
        }
    }
    return oss.str();
}

void DiagnosticAggregator::Aggregate(const SystemDiagnostics& diagnostics)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->diagnostics_ = diagnostics;
    this->diagnostics_.system_health = this->diagnostics_.ComputeSystemHealth();
}

std::string DiagnosticAggregator::GenerateDetailedReport() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->diagnostics_.GenerateReport();
}

std::string DiagnosticAggregator::GenerateOneLineSummary() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    std::ostringstream oss;
    bool first = true;
    for (const auto& entry : this->diagnostics_.components)
    {
        if (!first)
        {
            oss << " | ";
        }
        oss << ComponentSummary(entry.second);
        first = false;
    }
    return oss.str();
}

HealthStatus DiagnosticAggregator::GetSystemHealth() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->diagnostics_.ComputeSystemHealth();
}

bool DiagnosticAggregator::HasError() const
{
    const HealthStatus health = this->GetSystemHealth();
    return health == HealthStatus::Error || health == HealthStatus::Failed;
}

bool DiagnosticAggregator::HasWarning() const
{
    const HealthStatus health = this->GetSystemHealth();
    return health == HealthStatus::Warning || health == HealthStatus::Error ||
        health == HealthStatus::Failed;
}

SystemDiagnostics DiagnosticAggregator::GetSnapshot() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->diagnostics_;
}

}  // namespace rl_sar::diagnostics
