#include <iostream>
#include <string>

#include "rl_sar/diagnostics/diagnostic_aggregator.hpp"

namespace
{

using rl_sar::diagnostics::ComponentDiagnostics;
using rl_sar::diagnostics::DiagnosticAggregator;
using rl_sar::diagnostics::HealthStatus;
using rl_sar::diagnostics::SystemDiagnostics;

bool ExpectTrue(bool condition, const char* name)
{
    if (!condition)
    {
        std::cerr << name << " was not true\n";
        return false;
    }
    return true;
}

bool TestAggregateAndSummary()
{
    std::cout << "Testing DiagnosticAggregator summary... ";
    SystemDiagnostics system;

    ComponentDiagnostics body;
    body.component_name = "body_state";
    body.component_type = "UnitreeAdapter";
    body.health = HealthStatus::OK;
    body.message = "OK";
    system.AddComponent("body_state", body);

    ComponentDiagnostics arm;
    arm.component_name = "arm_state";
    arm.component_type = "ArxAdapter";
    arm.health = HealthStatus::Warning;
    arm.message = "tracking error elevated";
    system.AddComponent("arm_state", arm);
    system.total_warnings = 1;

    DiagnosticAggregator aggregator;
    aggregator.Aggregate(system);
    if (!ExpectTrue(aggregator.HasWarning(), "HasWarning")) { return false; }
    if (!ExpectTrue(!aggregator.HasError(), "HasError false")) { return false; }

    const std::string summary = aggregator.GenerateOneLineSummary();
    if (!ExpectTrue(summary.find("body_state") != std::string::npos, "summary body")) { return false; }
    if (!ExpectTrue(summary.find("arm_state") != std::string::npos, "summary arm")) { return false; }
    std::cout << "PASS\n";
    return true;
}

bool TestDetailedReport()
{
    std::cout << "Testing DiagnosticAggregator report... ";
    SystemDiagnostics system;
    ComponentDiagnostics supervisor;
    supervisor.component_name = "supervisor";
    supervisor.component_type = "Supervisor";
    supervisor.health = HealthStatus::Failed;
    supervisor.message = "fault latched";
    supervisor.details["mode"] = "FAULT_LATCHED";
    system.AddComponent("supervisor", supervisor);
    system.total_errors = 1;

    DiagnosticAggregator aggregator;
    aggregator.Aggregate(system);
    if (!ExpectTrue(aggregator.GetSystemHealth() == HealthStatus::Failed, "system health failed")) { return false; }
    const std::string report = aggregator.GenerateDetailedReport();
    if (!ExpectTrue(report.find("fault latched") != std::string::npos, "report message")) { return false; }
    if (!ExpectTrue(report.find("FAULT_LATCHED") != std::string::npos, "report detail")) { return false; }
    std::cout << "PASS\n";
    return true;
}

}  // namespace

int main()
{
    if (!TestAggregateAndSummary()) { return 1; }
    if (!TestDetailedReport()) { return 1; }
    std::cout << "All DiagnosticAggregator tests passed.\n";
    return 0;
}
