#-------------------------------------------------------------------------------
#   Assembly Hole Location (GKN Fokker)
#   Generic specification of a hole. This can be used to 
#   specify the locations to drill holes, where to place temporary fasteners, 
#   and/or where to place permanent fasteners. 
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                              # uid of the hole position

geometry_msgs/Pose nom_pos              # Nominal location of the hole

float32 diam                            # Diameter of the hole

float32 stack_t                         # Total thickness at the hole
                                        # Must be equal to the sum of layer thicknesses

float32 drill_jig_dist                  # distance of the drill jig hole entry above the surface

bool drilled                            # Whether or not the hole has been drilled

AssemblyMaterialLayer[] layers          # Material definition for each layer