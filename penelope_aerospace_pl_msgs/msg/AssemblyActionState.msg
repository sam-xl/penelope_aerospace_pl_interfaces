#-------------------------------------------------------------------------------

# Description of the Assembly Action states:
#   RECEIVED
#       ...
#   ACCEPTED  
#   if all equipment is available
#       ...
#   IN_PROGRESS
#       ...
#   PAUSED_WAITING
#       ...
#   WAITING_FOR_OPERATOR_ACTION
#       ...
#   SUCCESS
#       ...
#   CANCELLED
#       ...

uint8 RECEIVED = 1
uint8 ACCEPTED = 2
uint8 IN_PROGRESS = 3
uint8 PAUSED_WAITING = 4
uint8 WAITING_FOR_OPERATOR_ACTION = 5
uint8 SUCCESS = 6
uint8 CANCELLED = 7

uint8 data