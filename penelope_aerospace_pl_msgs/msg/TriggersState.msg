#-------------------------------------------------------------------------------
# DVM Triggers State (TNO)
#-------------------------------------------------------------------------------

# Description of the DVM process states:
#   TRIGGERS_ACTIVE
#     A motion request with triggers for the DVM system is sent to the manipulator
#     and is being processed.
#   NO_TRIGGERS
#     No motion request with triggers is active or being processed. This is a valid
#     state for motions without triggers or while no motion is requested.

uint8 TRIGGERS_ACTIVE = 1
uint8 NO_TRIGGERS = 2
uint8 data
