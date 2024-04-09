#-------------------------------------------------------------------------------
# Induction Welding State (KvE)
#-------------------------------------------------------------------------------

# Description of the module states:
#  PREWELD
#    The system is in the process of the coupling the end-effector to the gantry
#    
#  WELDING
#    The module is currently conducting a weld
#  POSTWELD
#    The gantry is moving the coil out of the tool to the hold position
#  PRESSURECOOL
#    The module is currently keeping the pressure on during cooling, until 
#    all the thermocouples will be below Tg
#  WELDABORTED
#    The welding has been aborted. This can be for several reasons, inluding too 
#    high temperatures or loss of air pressure.
#  SAFEOPERATORMODE
#    The machine is currently in its loading/unloading mode and cannot be 
#    used for welding. Wait until operator has closed and locked tool
#  READYTOWELD
#    The tool + process + machine + gantry are ready for welding
#  WAITING
#    The tool + process + machine are waiting, next steps could be "READYTOWELD" or "SAFEOPERATORMODE"



uint16 PREWELD=1
uint16 WELDING=2
uint16 POSTWELD=7
uint16 PRESSURECOOL=3
uint16 WELDABORTED=4
uint16 SAFEOPERATORMODE=5
uint16 READYTOWELD=6
uint16 WAITING=8



uint16 data