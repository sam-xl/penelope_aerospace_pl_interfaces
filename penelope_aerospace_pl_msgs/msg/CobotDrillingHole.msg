#-------------------------------------------------------------------------------
# Cobot Drilling Hole Specification (GKN Fokker)
#   Generic specification of a hole for cobot drilling. This can be used to 
#   specify the locations to drill holes, where to place temporary fasteners, 
#   and/or where to place permanent fasteners. 
#-------------------------------------------------------------------------------

# Locations of the hole
geometry_msgs/Pose pose 

# Hole specification
float32 diameter                        # Diameter of the hole in mmm

# Temporary fastener specifications
bool place_temp_fastener                # Indicate if temporary fastener needs to be placed

# uint8 TYPE_1 = 1
# uint8 TYPE_2 = 2
# ...# 
uint8 type                              # Type of temp fastener to place

# Drill specifications
float32 speed                           # Speed of the drill to use during the drilling
float32 feed                            # Feed rate of the drill
float32 depth                           # Depth of the hole to drill
uint16 number_of_layers                 # Number of layers in the stack
float32[] lower_torque_limits           # Lower limit(s) of the torque bandwidth(s)
float32[] upper_torque_limits           # Upper limit(s) of the torque bandwidth(s)
float32[] threshold                     # Torque level/threshold to switch to the next layer in the stack

# Permanent Fastener specifications
bool wet_installation                   # Indicate if wet installation is required
float32[2] fastener_torque_bandwidth    
