# Command a joint trajectory to the desired joint(s). Note that the commands are processed differently based on the currently set operating mode.
# For example, if the operating mode is set to 'position', the commands are interpreted as positions in radians

string cmd_type                               # set to 'single' for a single joint or 'group' for a group of joints
string name                                   # joint group name if 'cmd_type' is set to 'group' or joint name if 'cmd_type' is set to 'single'
trajectory_msgs/JointTrajectory traj          # ROS trajectory message
