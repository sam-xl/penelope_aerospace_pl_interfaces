#-------------------------------------------------------------------------------
#   Definition of an Assembly Action (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

#TODO ACTION_TYPE_TAG

string uid                  # uid of the hole Action

string obj_uid              # uid of the object
                            # Defines drilling action if the obj_uid is a drill object
                            # Defines temporary fastener movement if the obj_uid is a temporary fastener object
                            # Defines a movement if the obj_uid is a waypoint
                            # Defines a permanent fastener placement if the obj_uid is a permanent fastener
                            # Also defines which End Effector is suitable for this action

string loc_uid              # uid of the target location of the object
                                        
AssemblyActionState state   # Status of the action

string[] passing            # list of waypoints uids to pass
                            # e.g. to pass between pick up and place of a temporary fastener

uint8 speed                 # the speed as percentage of the maximum speed