# Health status of a single hardware component
# Published by: diagnostic_node (one message per component)
# Subscribed by: HMI diagnostic page
#
# Example component names:
# "lidar", "imu", "ultrasonic_front", "ultrasonic_right",
# "ultrasonic_back", "ultrasonic_left", "motor_left", "motor_right"

std_msgs/Header header

# Which component this report is about
string component_name

# Health status
int8 status
string status_message

# Latest sensor reading — for displaying live data on diagnostic page
float32 last_reading
string unit

# Status constants
int8 STATUS_OK=0
int8 STATUS_WARN=1
int8 STATUS_ERROR=2
int8 STATUS_OFFLINE=3
