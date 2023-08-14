#-------------------------------------------------------------------------------
# Cobot Drilling Hole State (GKN Fokker)
#-------------------------------------------------------------------------------

# Description of the Cobot Drilling process states:
# READY
#     ...
# INITIALIZING
#     ...
# RUNNING
#     ...
# RUNNING_COLABORATIVE
#     ...
# RUNNING_SERVICE_NEEDED
#     ...
# PAUSED
#     ...
# PAUSED_SAFETY
#     ...
# PAUSED_ALLOCATED_SPACE_LIMIT
#     ...
# SAFETY
#     ...

uint8 READY = 1
uint8 INITIALIZING = 2
uint8 RUNNING = 3
uint8 RUNNING_COLABORATIVE = 4
uint8 RUNNING_SERVICE_NEEDED = 5
uint8 PAUSED = 6
uint8 PAUSED_SAFETY = 7
uint8 PAUSED_ALLOCATED_SPACE_LIMIT = 8
uint8 SAFETY = 9

uint8 data
