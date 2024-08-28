#-------------------------------------------------------------------------------
# InspectNdt State
#-------------------------------------------------------------------------------

# Description of the process states:
#  INITIALIZE
#    Validate goal and ensure all systems are up and running.
#  RECONSTRUCT
#    Create a 3D reconstruction of the object to be inspected.
#  TOOL_PATH
#    Filter the 3D object mesh and generate the tool path.
#  INSPECT
#    Execute the inspection of the object.
#  POST_PROCESS
#    Process the inspection data.

uint8 INITIALIZE=1
uint8 RECONSTRUCT=2
uint8 TOOL_PATH=3
uint8 INSPECT=4
uint8 POST_PROCESS=5

uint8 data
