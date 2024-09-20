# This message is used specifically in the interbotix_xslocobot_joy package
#
# Maps raw 'joy' commands to more specific ones to control an Interbotix LoCoBot

# enum values that define the joystick controls for the robot

#########################################################################################################

# Reset base odometry
int8 RESET_ODOM = 1

#########################################################################################################

# Control the pan-and-tilt mechanism
int8 PAN_CCW = 2
int8 PAN_CW = 3
int8 TILT_UP = 4
int8 TILT_DOWN = 5
int8 PAN_TILT_HOME = 6

#########################################################################################################

# Control the motion of the virtual 'ee_gripper_link' or end effector using the modern_robotics_ik engine
# Position Control
int8 EE_X_INC = 7
int8 EE_X_DEC = 8
int8 EE_Y_INC = 9
int8 EE_Y_DEC = 10
int8 EE_Z_INC = 11
int8 EE_Z_DEC = 12

# Orientation Control
int8 EE_ROLL_CCW = 13
int8 EE_ROLL_CW = 14
int8 EE_PITCH_UP = 15
int8 EE_PITCH_DOWN = 16

#########################################################################################################

# Control the motion of independent joints on the Arm or send predefined robot poses
# Waist Joint Control
int8 WAIST_CCW = 17
int8 WAIST_CW = 18

# Gripper Control
int8 GRIPPER_RELEASE = 19
int8 GRIPPER_GRASP = 20

# Pose Control
int8 HOME_POSE = 21
int8 SLEEP_POSE = 22

#########################################################################################################

# Customize configurations for the Interbotix Arm
# Inc/Dec Joint speed
int8 SPEED_INC = 23
int8 SPEED_DEC = 24

# Quickly toggle between a fast and slow speed setting
int8 SPEED_COARSE = 25
int8 SPEED_FINE = 26

# Inc/Dec Gripper pressure
int8 GRIPPER_PWM_INC = 27
int8 GRIPPER_PWM_DEC = 28

#########################################################################################################

# Control the motion of the Kobuki base
float64 base_x_cmd
float64 base_theta_cmd
int8 base_reset_odom_cmd

# Control the motion of the camera pan-and-tilt mechanism
int8 pan_cmd
int8 tilt_cmd

# Control the motion of the Interbotix Arm
int8 ee_x_cmd
int8 ee_y_cmd
int8 ee_z_cmd
int8 ee_roll_cmd
int8 ee_pitch_cmd

# Independent Joint/Pose Control
int8 waist_cmd
int8 gripper_cmd
int8 pose_cmd

# Arm Configs
int8 speed_cmd
int8 speed_toggle_cmd
int8 gripper_pwm_cmd
