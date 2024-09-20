# This message is used specifically in the interbotix_rpi_modules package
#
# Configure NeoPixel Leds on a Raspberry Pi

string cmd_type       # Either 'pulse', 'blink', 'brightness', or 'color'.
                      # Note that the 'pulse' and 'brightness' options will
                      # affect all Leds even if 'set_all_leds' is set to False.
bool set_all_leds     # Set to True to configure all LEDs
uint8 pixel           # Index of NeoPixel to change (only considered if 'set_all_leds' is false)
uint32 color			    # Value indicates Color to be used (HEX - RGB)
uint8 brightness      # Value indicates LED brightness level (0-255)
uint32 period			    # time delay for blink or pulse (milliseconds)
uint8 iterations		  # iterations = number of iterations for blink or pulse
