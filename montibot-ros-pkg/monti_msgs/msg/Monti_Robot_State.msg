# Main state message containing information sent from the robot
uint8 header    # Information on current robot configuration
uint8 error     # Contains 1 of 8 documented error codes (00000000==no error)
int8[6] encoders
Misc_Sensor[10] modular_sensors