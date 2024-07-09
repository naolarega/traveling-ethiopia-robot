#!/usr/bin/env python3
import rclpy

from libs.nav_move import NavMove


def main(args=None):
    rclpy.init(args=args)
    speed_controller = NavMove()
    speed_controller.run()
    speed_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
