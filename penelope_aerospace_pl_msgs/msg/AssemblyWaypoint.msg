#-------------------------------------------------------------------------------
#   Definition of an Assembly Waypoint (GKN Fokker)
#   
#   Waypoints can be used to manage the cobot movements
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                  # uid of the hole Waypoint

geometry_msgs/Pose pos      # location of the waypoint

bool orient                 # whether or not the orientation is mandatory

float32 blend_radius        # Whether the cobot movement can cut a corner when
                            # passing this waypoint.
                            # A value of zero means that no corner must be cut
                            # Notice that the cobot program will excute until the 
                            # next move statement. Specification of a non-zero
                            # value risky should be done cautiously.