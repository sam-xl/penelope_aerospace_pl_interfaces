#-------------------------------------------------------------------------------
#   Definition of an End Effector (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                      # uid of the End Effector

string loc_uid                  # uid of the location of the End Effector in one of
                                # possible docking positions

string poss_dock_pos_uid[]      # list of uid's of possible docking positions

AssemblyEeState state           # The state of the End Effector