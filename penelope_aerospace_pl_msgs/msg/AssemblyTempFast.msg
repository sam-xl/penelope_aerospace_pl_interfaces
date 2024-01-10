#-------------------------------------------------------------------------------
#   Definition of a temporary fastener (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                              # uid of the temporary fastener

string loc_uid                          # uid of the location of the temporary fastener

string ee_uid                           # uid of the end effector needed to manipulate this temporary fastener

AssemblyFastState state                 # The state of the temporary fastener

geometry_msgs/Pose inst_pos             # Installed location of the temporary fastener
                                        # Only available after installation

float32 diam                            # diameter of the temporary fastener

float32 shaft_height                    # the height of the shaft that is sticking out when installed

float32 min_stack                       # the minimum stack

float32 max_stack                       # the maximum stack

float32 tcp_tip_distace                 # distance between hole entry point and tip if inserted in a hole 

float32 tcp_top_distace                 # distance between hole entry point and top if inserted in a hole
                                        # the top is where the tcp is when engaging the tempf 