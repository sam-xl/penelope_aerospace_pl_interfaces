#-------------------------------------------------------------------------------
#   Assembly Hole Location Container (GKN Fokker)
#   Can be a product or a storage location. 
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                              # uid of the container

AssemblyHoleLocation[] locations        # List of hole locations

float32 max_obstacle_height             # maximum obstacle height
                                        # e.g. temporary fasteners sticking out