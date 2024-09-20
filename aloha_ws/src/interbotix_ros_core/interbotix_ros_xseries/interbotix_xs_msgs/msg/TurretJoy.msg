# This message is used specifically in the interbotix_xsturret_simple_interface package
#
# Maps raw 'joy' commands to more specific ones to control an Interbotix turret

# enum values that define the joystick controls for the robot

#########################################################################################################

# Control the pan-and-tilt mechanism
int8 PAN_CCW = 1
int8 PAN_CW = 2
int8 TILT_UP = 3
int8 TILT_DOWN = 4
int8 PAN_TILT_HOME = 5

#########################################################################################################

# Customize configurations for the Interbotix Turret
# Inc/Dec Joint speed
int8 SPEED_INC = 6
int8 SPEED_DEC = 7

# Quickly toggle between a fast and slow speed setting
int8 SPEED_COARSE = 8
int8 SPEED_FINE = 9

#########################################################################################################

# Control the motion of the pan-and-tilt mechanism
int8 pan_cmd
int8 tilt_cmd

# Speed Configs
int8 speed_cmd
int8 speed_toggle_cmd
