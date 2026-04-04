# Diagnostic metrics for sim2real validation
# This message contains all key metrics required for monitoring
# system health and detecting simulation-to-real gaps

# Header for timestamp and frame info
std_msgs/Header header

# Policy execution metrics
uint64 policy_latency_us          # Time from observation to action output (microseconds)
float64 policy_frequency_hz       # Measured policy execution frequency

# Coordinator metrics
uint64 coordinator_jitter_us      # Deviation from expected coordinator timing (microseconds)
float64 coordinator_frequency_hz  # Measured coordinator loop frequency

# State freshness metrics
uint64 body_state_age_us          # Age of body state from source (microseconds)
uint64 arm_state_age_us           # Age of arm state from backend (microseconds)

# Arm tracking metrics
float64 arm_tracking_error_norm   # L2 norm of joint tracking error
bool arm_tracking_healthy         # Overall arm tracking health flag

# Drift metrics (for zero-velocity validation)
float64 xy_drift_error            # Euclidean XY drift from reference (meters)
float64 yaw_drift_error           # Absolute yaw drift from reference (radians)

# Safety indicators
int32 clip_count                  # Number of safety clip events
int32 seq_gap_count               # Number of sequence gaps detected

# Mode information
string current_mode               # Current operational mode string
uint32 mode_transition_count      # Total number of mode transitions

# Overall health status
bool is_degraded                 # True if any critical metric is outside healthy bounds
