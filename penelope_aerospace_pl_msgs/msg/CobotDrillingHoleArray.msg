#-------------------------------------------------------------------------------
# Cobot Drilling Hole Array (GKN Fokker)
#   Definition of the holes to drill including if temporary fastener needs to
#   be placed after.
#-------------------------------------------------------------------------------

# Header containing the timestamp and frame id
std_msgs/Header header

# Specification of the holes to drill
CobotDrillingHole[] holes

# Indicate if a temporary fastener needs to be placed after drilling
bool[] place_temp_fastener
float32[] temp_fastener_placement_torque
