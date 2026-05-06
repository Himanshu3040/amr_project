# Processed ultrasonic zone data
# Published by: obstacle_monitor_node
# Subscribed by: nav2 stack, delivery_ack_node, diagnostic_node
#
# Zone definitions (based on DYP-A21 range: 3cm to 500cm):
# CLEAR      = distance > 300cm  → robot moves normally
# WARNING    = 150cm to 300cm   → robot slows down
# PROTECTIVE = 3cm to 150cm     → robot stops, waits 120 seconds

std_msgs/Header header

# Raw distances (copied from UltrasonicRaw for reference)
float32[4] distances_mm

# Zone classification per sensor
# 0=CLEAR, 1=WARNING, 2=PROTECTIVE
int8[4] zones

# Overall robot zone — worst (highest) zone across all 4 sensors
# This is what Nav2 and obstacle logic actually acts on
int8 robot_zone

# How many seconds the robot has been stopped in PROTECTIVE zone
# Resets to 0 when obstacle clears or robot reroutes
float32 stop_duration_seconds

# Zone constants — use these in your code by name, not by number
int8 ZONE_CLEAR=0
int8 ZONE_WARNING=1
int8 ZONE_PROTECTIVE=2
