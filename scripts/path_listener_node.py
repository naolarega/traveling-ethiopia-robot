#!/usr/bin/env python3
import rclpy

from libs.path_listener import PathListenerNode

def main(args=None):
	rclpy.init(args=args)
	node=PathListenerNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__=="__main__":
	main()

