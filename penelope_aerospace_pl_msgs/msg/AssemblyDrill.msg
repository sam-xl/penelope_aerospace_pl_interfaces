#-------------------------------------------------------------------------------
#   Definition of a drill operation (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                              # uid of the drill operation

string loc_uid                          # uid of the location of the hole in one of the containers

string ee_uid                           # uid of the end effector needed drill this hole

float32 diam                            # diameter of the drill
                                        # for reference only because real diameter defined by ee_uid

geometry_msgs/Pose jig_pos              # Location of the drill jig drill guiding hole
                                        # Only available after drilling

AssemblyMaterialLayer[] layers          # Material layers as encountered during drilling
                                        # Data filled during and_or after drilling
