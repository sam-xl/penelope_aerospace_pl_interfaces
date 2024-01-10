#-------------------------------------------------------------------------------
#   Definition of an Assembly Action (GKN Fokker)
#
#   All locations in global axis system
#   All dimensions in mm
#-------------------------------------------------------------------------------

string uid                  # uid of the hole Action

string obj_uid              # uid of the object
                            # Defines drilling action if the obj_uid is a drill object
                            # Defines temporary fastener movement if the obj_uid is a temporary fastener object
                            # Defines a movement if the obj_uid is a waypoint
                            # Defines a permanent fastener placement if the obj_uid is a permanent fastener

string loc_uid              # uid of the target location of the object

string ee_uid               # uid of the end effector to use for this action
                            # ee_uid must fit the obj_uid (e.g. drill end effector for drill task)
                                        
AssemblyActionState state   # Status of the action

string[] passing            # list of waypoints uids to pass
                            # e.g. to pass between pick up and place of a temporary fastener

uint8 speed                 # the speed as percentage of the maximum speed