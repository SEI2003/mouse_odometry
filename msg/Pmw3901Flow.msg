# One PMW3901 delta accumulated over integration_time.
# header.stamp is the end of that integration interval. The producer preserves
# the sensor-axis convention; the odometry node applies calibration and yaw.
std_msgs/Header header
int32 delta_x_count
int32 delta_y_count
builtin_interfaces/Duration integration_time
bool valid
bool quality_available
uint16 quality
