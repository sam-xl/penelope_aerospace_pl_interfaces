#-------------------------------------------------------------------------------
#   Definition of an Assembly Agent (GKN Fokker)
#
#   An Assembly Agent contains information needed for certain actions
#   Actions can be allocated to it and executed
#-------------------------------------------------------------------------------

string uid                                  # uid of the Agent

AssemblyHoleLocationContainer[] storage     # storage location containers

AssemblyHoleLocationContainer[] products    # product containers with holes

AssemblyWaypoint[] Waypoints                # list of defined waypoints

AssemblyAction[] actions                    # list of defined actions

AssemblyDrill[] drills                      # list of available drills

AssemblyFast[] fasteners                    # list of available fasteners

AssemblyTempFast[] tempfs                   # list of available temporary fasteners
