# Command the joints in the specified joint group. Note that the commands are processed differently based on the group's operating mode.
# For example, if a group's operating mode is set to 'position', the commands are interpreted as positions in radians

string name            # Name of joint group
float32[] cmd          # List of joint commands; order is dictated by the index of each joint name for the given group in the 'groups' section of a 'motor_config' yaml file
