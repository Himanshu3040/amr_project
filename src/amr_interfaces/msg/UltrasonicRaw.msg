# Raw ultrasonic distances from STM32 via Ethernet UDP
# Published by: ethernet_bridge_node
# Subscribed by: ultrasonic_safety_stop_node
#
# Sensor index mapping:
#   Index 0 = Front Left
#   Index 1 = Front Right
#   Index 2 = Rear Left
#   Index 3 = Rear Right
#
# All distances in millimetres (mm)
# Valid range: 30mm to 5000mm (DYP-A21 spec)

std_msgs/Header header
float32[4] distances_mm