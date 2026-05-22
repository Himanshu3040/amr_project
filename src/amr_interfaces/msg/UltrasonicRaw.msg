# Raw ultrasonic distances from STM32 via Ethernet UDP
# Published by: ethernet_bridge_node
# Subscribed by: obstacle_monitor_node
#
# Sensor positions:
# Index 0 = Front sensor
# Index 1 = Right sensor
# Index 2 = Back sensor
# Index 3 = Left sensor

std_msgs/Header header

float32[4] distances_mm
