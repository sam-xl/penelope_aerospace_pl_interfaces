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
geometry_msgs/Pose inst_pos             # Installed location of the hole

float32 diam                            # Diameter of the hole

float32 stack_t                         # Total thickness at the hole
                                        # Must be equal to the sum of layer thicknesses

AssemblyMaterialLayer[] layers          # Material definition for each layer