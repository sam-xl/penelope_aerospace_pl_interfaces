#-------------------------------------------------------------------------------
# Module State
#-------------------------------------------------------------------------------

# Description of the module states:
#  ACCEPTED
#    The goal has been received and is accepted
#  EXECUTING
#    The module is executing the request
#  PAUSED
#    The module is paused by the pause service and waiting to continue
#  CANCELING
#    The module received a cancel request and is stopping execution

uint8 ACCEPTED=1
uint8 EXECUTING=2
uint8 PAUSED=3
uint8 CANCELING=4

uint8 data
