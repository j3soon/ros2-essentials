# This message is used specifically in the interbotix_xshexapod_joy package
#
# Maps raw 'joy' commands to more specific ones to control an Interbotix hexapod

# enum values that define the joystick controls for the robot

#########################################################################################################

# Command Options to move the Hexapod in the 'world' or in 'place'
# World Control
int8 WORLD_X_INC = 1
int8 WORLD_X_DEC = 2
int8 WORLD_Y_INC = 3
int8 WORLD_Y_DEC = 4
int8 WORLD_YAW_CCW = 5
int8 WORLD_YAW_CW = 6

# In Place Control
int8 PLACE_X_INC = 7
int8 PLACE_X_DEC = 8
int8 PLACE_Y_INC = 9
int8 PLACE_Y_DEC = 10
int8 PLACE_Z_INC = 11
int8 PLACE_Z_DEC = 12
int8 PLACE_ROLL_CCW = 13
int8 PLACE_ROLL_CW = 14
int8 PLACE_PITCH_UP = 15
int8 PLACE_PITCH_DOWN = 16

# Move Type
int8 MOVE_HEXAPOD = 17
int8 MOVE_LEG = 18

# Pose Control
int8 HOME_POSE = 19
int8 SLEEP_POSE = 20

#########################################################################################################

# Customize configurations for the Interbotix Hexapod

# Cycle through the various gaits when in 'move_hexapod' mode
int8 GAIT_NEXT = 21
int8 GAIT_PREVIOUS = 22

# Cycle through the legs when in 'move_leg' mode
int8 LEG_NEXT = 23
int8 LEG_PREVIOUS = 24

# Widen or narrow the hexapod's stance
int8 WIDEN_STANCE = 25
int8 NARROW_STANCE = 26

# Reboot motors if necessary
int8 REBOOT_MOTORS = 27

# Set the current stance and hexapod height to be the new 'Home Pose'
int8 SET_HOME_POSE = 28

# Inc/Dec speed
int8 SPEED_INC = 29
int8 SPEED_DEC = 30

# Quickly toggle between a fast and slow speed setting
int8 SPEED_COARSE = 31
int8 SPEED_FINE = 32

#########################################################################################################

# Control the motion of the Interbotix Hexapod
int8 world_x_cmd
int8 world_y_cmd
int8 world_yaw_cmd

int8 place_x_cmd
int8 place_y_cmd
int8 place_z_cmd
int8 place_roll_cmd
int8 place_pitch_cmd

int8 pose_cmd
int8 move_type_cmd

# Other Options
int8 gait_toggle_cmd
int8 leg_toggle_cmd
int8 stance_cmd
int8 reboot_cmd
int8 set_home_pose_cmd

# Speed Configs
int8 speed_cmd
int8 speed_toggle_cmd
