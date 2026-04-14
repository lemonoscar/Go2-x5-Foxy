#include "rl_sar/diagnostics/diagnostics_publisher.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif  // USE_ROS2

namespace rl_sar::diagnostics
{

DiagnosticsPublisher::DiagnosticsPublisher() = default;

DiagnosticsPublisher::~DiagnosticsPublisher()
{
    Stop();
}

bool DiagnosticsPublisher::Initialize(const Config& config)
{
    std::lock_guard<std::mutex> lock(start_stop_mutex_);

    if (initialized_.load()) {
        // Already initialized
        return true;
    }

    // Validate configuration
    if (!config.IsValid()) {
        return false;
    }

    config_ = config;

#ifdef USE_ROS2
    // Initialize ROS2 components if enabled
    if (config_.ros2_enabled) {
        try {
            // Create ROS2 node with default options
            rclcpp::NodeOptions node_options;
            node_options.start_parameter_event_publisher(false);
            node_options.start_parameter_services(false);

            ros_node_ = std::make_shared<rclcpp::Node>(config_.node_name, node_options);

            // Create publisher for diagnostics
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.reliable();
            qos.durability_volatile();

            diagnostics_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
                config_.diagnostics_topic, qos);

        } catch (const std::exception& e) {
            // ROS2 initialization failed
            return false;
        }
    }
#else
    if (config_.ros2_enabled) {
        // ROS2 requested but not available
        return false;
    }
#endif  // USE_ROS2

    // Initialize metrics
    metrics_buffer_.Reset();
    last_publish_time_ = std::chrono::steady_clock::now();

    initialized_.store(true);
    return true;
}

bool DiagnosticsPublisher::Start()
{
    std::lock_guard<std::mutex> lock(start_stop_mutex_);

    if (!initialized_.load()) {
        return false;
    }

    if (running_.load()) {
        // Already running
        return true;
    }

    should_stop_.store(false);
    publish_count_.store(0);
    failed_publish_count_.store(0);

    // Launch publish thread
    publish_thread_ = std::thread(&DiagnosticsPublisher::PublishThread, this);

    running_.store(true);
    return true;
}

void DiagnosticsPublisher::Stop()
{
    std::lock_guard<std::mutex> lock(start_stop_mutex_);

    if (!running_.load()) {
        return;
    }

    should_stop_.store(true);
    cv_.notify_all();

    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }

    running_.store(false);
}

void DiagnosticsPublisher::UpdateMetrics(const DiagnosticMetrics& metrics)
{
    std::unique_lock<std::shared_mutex> lock(metrics_mutex_);

    // Update metrics buffer
    metrics_buffer_ = metrics;

    // Update timestamp to current time if not provided
    if (metrics_buffer_.timestamp_ns == 0) {
        metrics_buffer_.timestamp_ns = GetMonotonicTimestamp();
    }
}

bool DiagnosticsPublisher::GetMetrics(DiagnosticMetrics& metrics) const
{
    std::shared_lock<std::shared_mutex> lock(metrics_mutex_);

    if (!initialized_.load()) {
        return false;
    }

    metrics = metrics_buffer_;
    return true;
}

DiagnosticsPublisher::Config DiagnosticsPublisher::GetConfig() const
{
    return config_;
}

DiagnosticsPublisher::PublishStats DiagnosticsPublisher::GetStats() const
{
    PublishStats stats;
    stats.total_publish_count = publish_count_.load();
    stats.failed_publish_count = failed_publish_count_.load();

    std::lock_guard<std::mutex> lock(stats_mutex_);

    if (stats.total_publish_count > 1) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            now - last_publish_time_).count();

        if (elapsed > 0.0) {
            // Estimate frequency from recent publishes
            stats.actual_frequency_hz = 1.0 / elapsed;
        }
    }

    return stats;
}

void DiagnosticsPublisher::ResetStats()
{
    publish_count_.store(0);
    failed_publish_count_.store(0);
    last_publish_time_ = std::chrono::steady_clock::now();
}

void DiagnosticsPublisher::PublishThread()
{
    const std::chrono::microseconds publish_interval(
        1000000 / config_.publish_rate_hz);

    while (!should_stop_.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // Get current metrics
        DiagnosticMetrics metrics;
        {
            std::shared_lock<std::shared_mutex> lock(metrics_mutex_);
            metrics = metrics_buffer_;
        }

        // Publish metrics
        bool success = PublishViaROS2(metrics);

        // Update statistics
        publish_count_.fetch_add(1);
        if (!success) {
            failed_publish_count_.fetch_add(1);
        }

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            last_publish_time_ = std::chrono::steady_clock::now();
        }

        // Calculate sleep time to maintain publish rate
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time);

        auto sleep_time = publish_interval - elapsed;

        if (sleep_time.count() > 0) {
            std::unique_lock<std::shared_mutex> lock(metrics_mutex_);
            cv_.wait_for(lock, sleep_time, [this] {
                return should_stop_.load();
            });
        }
    }
}

bool DiagnosticsPublisher::PublishViaROS2(const DiagnosticMetrics& metrics)
{
#ifdef USE_ROS2
    if (config_.ros2_enabled && diagnostics_pub_) {
        try {
            auto msg = std_msgs::msg::String();
            msg.data = MetricsToJSON(metrics);

            diagnostics_pub_->publish(msg);
            return true;

        } catch (const std::exception& e) {
            return false;
        }
    }
#endif  // USE_ROS2

    // Standalone mode - no ROS2 publishing
    return !config_.ros2_enabled;
}

std::string DiagnosticsPublisher::MetricsToJSON(const DiagnosticMetrics& metrics)
{
    std::ostringstream json;

    json << "{";
    json << "\"timestamp_ns\":" << metrics.timestamp_ns << ",";
    json << "\"status\":\"" << metrics.GetStatusString() << "\",";
    json << "\"policy\":{";
    json << "\"latency_us\":" << metrics.policy_latency_us << ",";
    json << "\"frequency_hz\":" << std::fixed << std::setprecision(2) << metrics.policy_frequency_hz << ",";
    json << "\"age_us\":" << metrics.policy_age_us << ",";
    json << "\"seq\":" << metrics.policy_seq << ",";
    json << "\"fresh\":" << (metrics.policy_fresh ? "true" : "false") << ",";
    json << "\"from_fresh_sample\":" << (metrics.policy_from_fresh_sample ? "true" : "false");
    json << "},";
    json << "\"coordinator\":{";
    json << "\"jitter_us\":" << metrics.coordinator_jitter_us << ",";
    json << "\"frequency_hz\":" << std::fixed << std::setprecision(2) << metrics.coordinator_frequency_hz;
    json << "},";
    json << "\"state_freshness\":{";
    json << "\"body_age_us\":" << metrics.body_state_age_us << ",";
    json << "\"arm_age_us\":" << metrics.arm_state_age_us;
    json << "},";
    json << "\"arm_tracking\":{";
    json << "\"error_norm\":" << std::fixed << std::setprecision(6) << metrics.arm_tracking_error_norm << ",";
    json << "\"healthy\":" << (metrics.arm_tracking_healthy ? "true" : "false");
    json << "},";
    json << "\"arm_backend\":{";
    json << "\"name\":\"" << metrics.arm_backend_name << "\",";
    json << "\"healthy\":" << (metrics.arm_backend_healthy ? "true" : "false") << ",";
    json << "\"age_us\":" << metrics.arm_backend_age_us;
    json << "},";
    json << "\"drift\":{";
    json << "\"xy_error_m\":" << std::fixed << std::setprecision(6) << metrics.xy_drift_error << ",";
    json << "\"yaw_error_rad\":" << std::fixed << std::setprecision(6) << metrics.yaw_drift_error << ",";
    json << "\"metrics_valid\":" << (metrics.drift_metrics_valid ? "true" : "false") << ",";
    json << "\"raw_data_available\":" << (metrics.drift_raw_data_available ? "true" : "false") << ",";
    json << "\"raw_sample_count\":" << metrics.drift_raw_sample_count << ",";
    json << "\"window_summary\":\"" << metrics.drift_window_summary << "\"";
    json << "},";
    json << "\"aggregated\":{";
    json << "\"health\":\"" << metrics.system_health << "\",";
    json << "\"summary\":\"" << metrics.system_summary << "\"";
    json << "},";
    json << "\"safety\":{";
    json << "\"clip_count\":" << metrics.clip_count << ",";
    json << "\"seq_gap_count\":" << metrics.seq_gap_count;
    json << "},";
    json << "\"mode\":{";
    json << "\"current\":\"" << metrics.current_mode << "\",";
    json << "\"transition_count\":" << metrics.mode_transition_count;
    json << "}";

    // Add degraded flag explicitly for easier filtering
    json << ",\"is_degraded\":" << (metrics.IsDegraded() ? "true" : "false");

    json << "}";

    return json.str();
}

uint64_t DiagnosticsPublisher::GetMonotonicTimestamp()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

double DiagnosticsPublisher::ComputeFrequency(
    const std::chrono::steady_clock::time_point& last_time,
    const std::chrono::steady_clock::time_point& current_time)
{
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        current_time - last_time).count();

    if (elapsed > 0.0) {
        return 1.0 / elapsed;
    }
    return 0.0;
}

}  // namespace rl_sar::diagnostics
