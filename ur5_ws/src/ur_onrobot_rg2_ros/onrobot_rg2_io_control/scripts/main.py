#!/usr/bin/env python3

import rclpy
from onrobot_rg2_io_control.onrobot_rg2_io_control import OnrobotRG2IOControl


def main(args=None):
    rclpy.init(args=args)

    onrobot_rg2_io_control = OnrobotRG2IOControl()

    try:
        rclpy.spin(onrobot_rg2_io_control)
    except SystemExit:
        pass
    finally:
        onrobot_rg2_io_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
