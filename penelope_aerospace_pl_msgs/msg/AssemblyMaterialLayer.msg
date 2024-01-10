#-------------------------------------------------------------------------------
#   Assembly Material Layer definition (GKN Fokker)
#-------------------------------------------------------------------------------

string uid                              # uid of the layer

float32 thickness                       # Thickness of the layer

# Drill specifications during the drilling in this layer
float32 speed                           # Speed of the drill
float32 feed                            # Feed rate of the drill
float32 lower_torque_limits             # Lower limit(s) of the torque bandwidth(s)
float32 upper_torque_limits             # Upper limit(s) of the torque bandwidth(s)
float32 threshold                       # Torque level/threshold to switch to the next layer in the stack

# Temporary fastening specifications for this layer
float32 max_clamp_force                 # Maximum temporary fastener clamp force