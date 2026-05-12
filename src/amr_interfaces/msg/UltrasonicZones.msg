# Processed ultrasonic zone data
# Published by: ultrasonic_safety_stop_node
# Subscribed by: safety_arbitrator_node, diagnostic_node
#
# Sensor index mapping:
#   Index 0 = Front Left
#   Index 1 = Front Right
#   Index 2 = Rear Left
#   Index 3 = Rear Right
#
# Zone definitions:
#   CLEAR      = distance > protective threshold  → robot moves normally
#   PROTECTIVE = distance <= protective threshold → robot stops immediately
#
# Protective thresholds:
#   Front sensors (index 0, 1): 250 mm
#   Rear  sensors (index 2, 3): 130 mm

std_msgs/Header header

# Raw distances copied from UltrasonicRaw for reference (mm)
float32[4] distances_mm

# Zone classification per sensor: 0=CLEAR, 1=PROTECTIVE
int8[4] zones

# Overall robot zone — worst across all 4 sensors
# If ANY sensor is PROTECTIVE, this is PROTECTIVE
int8 robot_zone

# Zone constants
int8 ZONE_CLEAR=0
int8 ZONE_PROTECTIVE=1