#-------------------------------------------------------------------------------
# Result codes
#
#   It is possible to either extend this message definition or to create a
#   module specific `ResultCodes.msg` file. 
#-------------------------------------------------------------------------------

# Generic OK
string SUCCES=""
uint16 RC_SUCCES=1

# Generic failure message
string FAILED="Action/Service failed scan with errors."
uint16 RC_FAILED=2

#-------------------------------------------------------------------------------
# Below specific result codes can be added for anticipated errors that can occur
# during runtime. Please follow the following structure:
#
#    # Comment explaining the error/failure message
#    string X_DISCONNECTED="Description and cause of error to show to operator"
#    uint16 RC_X_DISCONNECTED=1234
#
# Always use the Generic OK message to indicate that your Action or Service was
# sucessful. The generic failure message should only be used to indicate events
# that were not anticipated.
#-------------------------------------------------------------------------------

# DVM status messages (Range between 100 and 199)
string DVM_FAILED_BY_USER="DVM scan failed, terminated by operator."
uint16 RC_DVM_FAILED_BY_USER=100
