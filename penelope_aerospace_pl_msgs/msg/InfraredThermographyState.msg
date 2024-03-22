#-------------------------------------------------------------------------------
# InfraredThermoMeasureState (NLR)
#-------------------------------------------------------------------------------

# Description of the process states:
#  STANDARD
#    Nothing is happening
#  GANTRY MOVING
#    The gantry is moving, independent of a measurement
#  MOVING TO POSE
#    Gantry is moving to a measurement location
#  MEASURING
#    Thermography measurement has been initiated. The module is currently conducting a measurement
#  MEASUREMENT COMPLETED
#    Thermography measurement is completed, measurement file created.
#  MEASUREMENT FAILED
#    Thermography measurement is failed, no measurement file has been created
#  MEASUREMENT ABORTED
#    Thermography measurement is aborted


uint16 STANDARD=0
uint16 GANTRY_MOVING=1
uint16 MOVING_TO_POSE=2
uint16 MEASURING=3
uint16 MEASUREMENT_COMPLETED=4
uint16 MEASUREMENT_FAILED=5
uint16 MEASUREMENT_ABORTED=6

uint16 data
