# Overall robot system status
# Published by: delivery_ack_node (and future status_manager_node)
# Subscribed by: HMI, logging, diagnostic_node

std_msgs/Header header

# Current operational state of the robot
int8 state

# Navigation information
string current_goal_id
float32 distance_to_goal_m

# Battery information (will be populated when battery monitor is added)
float32 battery_voltage
float32 battery_percentage

# Delivery / task acknowledgement
bool goal_reached
string ack_message

# State constants
# Use these names in your code — never hardcode the numbers
int8 STATE_IDLE=0
int8 STATE_MAPPING=1
int8 STATE_NAVIGATING=2
int8 STATE_OBSTACLE_WAIT=3
int8 STATE_REROUTING=4
int8 STATE_GOAL_REACHED=5
int8 STATE_ERROR=6
