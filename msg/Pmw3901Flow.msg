# One PMW3901 delta accumulated over integration_time.
# header.stamp is the measurement-end timestamp, not the transmission time.
# Set it to zero only when the adapter cannot provide a measurement timestamp;
# the odometry node then uses ROS receive time as an explicitly marked fallback.
# The producer preserves the sensor-axis convention; the node calibrates it.
std_msgs/Header header
uint32 cycle_id
int32 delta_x_count
int32 delta_y_count
builtin_interfaces/Duration integration_time
bool valid
bool quality_available
uint16 quality
