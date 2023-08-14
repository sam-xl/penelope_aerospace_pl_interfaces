#-------------------------------------------------------------------------------
# InfraredThermoMeasureState (NLR)
#-------------------------------------------------------------------------------

# Description of the module states:
#  HEATING
#    The IR camera is heating up
#  MEASURING
#    The module is currently conducting a measurement
#  COOLING
#    The IR camera is cooling off (8 minutes)
#  VIBRATION
#    The module is currently experiencing this level of vibrations 

uint16 HEATING_UP=1
uint16 MEASUREMENT_IN_PROGRESS=2
uint16 COOLING_DOWN=3
uint16 VIBRATION=4

uint16 data
