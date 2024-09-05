# Command a desired joint. Note that the command is processed differently based on the joint's operating mode.
# For example, if a joint's operating mode is set to 'position', the command is interpreted as a position in radians

string name          # Name of joint
float32 cmd          # Joint command
