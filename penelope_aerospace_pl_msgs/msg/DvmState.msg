#-------------------------------------------------------------------------------
# DVM State (TNO)
#-------------------------------------------------------------------------------

# Description of the DVM process states:
#   SUCCESS
#       ...
#   IDLE
#       ...
#   WAITING_FOR_OPERATOR_ACTION
#       ...
#   INITIALIZING
#       ...
#   MEASURING
#       ...
#   PROCESSING
#       ...
#   FINALIZING
#       ...
#   CANCELLED
#       ...
#   ABORTED
#       ...
#   FAILED
#       ...
#   OFFLINE
#       ...

uint8 SUCCESS = 1
uint8 IDLE = 2
uint8 WAITING_FOR_OPERATOR_ACTION = 3
uint8 INITIALIZING = 4
uint8 MEASURING = 5
uint8 PROCESSING = 6
uint8 FINALIZING = 7
uint8 CANCELLED = 8
uint8 ABORTED = 9
uint8 FAILED = 10
uint8 OFFLINE = 11

uint8 data
