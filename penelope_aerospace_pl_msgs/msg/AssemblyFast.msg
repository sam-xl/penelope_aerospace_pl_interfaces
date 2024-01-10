#-------------------------------------------------------------------------------
#   Definition of a fastener (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                              # uid of the fastener

string loc_uid                          # uid of the location of the fastener

float32 diam                            # diameter of the fastener
float32 shaft_height                    # the height of the shaft that is sticking out when in storage location
float32 min_stack                       # the minimum stack
float32 max_stack                       # the maximum stack
float32 tcp_tip_distace                 # distance between hole entry point and tip if inserted in a hole 
float32 tcp_top_distace                 # distance between hole entry point and top if inserted in a hole
                                        # the top is where the tcp is when engaging the tempf 