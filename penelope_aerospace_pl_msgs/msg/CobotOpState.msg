#-------------------------------------------------------------------------------
# Cobot Operation State (GKN Fokker)
#-------------------------------------------------------------------------------

# Description of the Cobot Operation states:
#   QUEUED
#       ...
#   PROCESSED
#       ...
#   IN_PROGRESS
#       ...
#   SUCCESS
#       ...
#   SKIPPED
#       ...
#   CANCELLED
#       ...
#   INCORRECTLY_PROCESSED
#       ...

uint8 QUEUED = 1
uint8 PROCESSED = 2
uint8 IN_PROGRESS = 3
uint8 SUCCESS = 4
uint8 SKIPPED = 5
uint8 CANCELLED = 6
uint8 INCORRECTLY_PROCESSED = 7

uint8 data
