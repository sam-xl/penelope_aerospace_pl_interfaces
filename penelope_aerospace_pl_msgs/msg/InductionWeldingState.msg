#-------------------------------------------------------------------------------
# Induction Welding State (KvE)
#-------------------------------------------------------------------------------

# Description of the module states:
#  PREWELD
#    The gantry is moving the coil to the starting position of the weld path
#  WELDING
#    The module is currently conducting a weld
#  POSTWELD
#    The gantry is moving the coil out of the tool to the hold position
#  PRESSURECOOL
#    The module is currently keeping the pressure on during cooling, until 
#    all the thermocouples will be below Tg

uint16 PREWELD=1
uint16 WELDING=2
uint16 POSTWELD=7
uint16 PRESSURECOOL=3

uint16 data
