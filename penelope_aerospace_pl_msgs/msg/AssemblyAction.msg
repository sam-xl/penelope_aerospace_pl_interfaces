#-------------------------------------------------------------------------------
#   Definition of an Assembly Action (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

#TODO ACTION_TYPE_TAG

string uid                  # uid of the hole Action

string a_type               # type of action
                            # ACTION_TYPE_MOVE_WAYPOINT = "move_to_waypoint"   
                            # ACTION_TYPE_INSTALL = "install_fastener"  
                            # ACTION_TYPE_REMOVE = "remove_fastener"  

string loc_uid              # uid of the target location of the object
                                        
AssemblyActionState state   # Status of the action

string[] passing            # list of waypoints uids to pass
                            # e.g. to pass between pick up and place of a temporary fastener

uint8 speed                 # the speed as percentage of the maximum speed