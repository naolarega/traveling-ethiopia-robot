#!/usr/bin/env python3
from two_wheeled_robot.msg import Path, Goal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Point,Pose2D
from two_wheeled_robot.msg import Path
from rclpy.node import Node

from math import atan2, sqrt
from queue import Queue
class PathListenerNode(Node):
	def __init__(self):
		super().__init__("path_listener_node")
		self.get_logger().info("listening_path")
		self.current_pose=None
		self.waypoints = Queue()
		self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
		
		self.timer = self.create_timer(0.8, self.timer_callback)
		
		self.subscription_path = self.create_subscription(
            Path,
            'path_topic',
            self.path_callback,
            10
        )

		self.subscription_pose = self.create_subscription(
			Odometry,
			'/wheel/odometry',
			self.pose_callback,
			10
		)
	def timer_callback(self):
		if not self.waypoints.empty():
			self.move_to_next_waypoint()
		else:
			self.stop_robot()
	def path_callback(self, msg):
		# Process the received path
		self.get_logger().info('Received path: %s' % msg)
		self.waypoints.queue.clear()  # Clear existing waypoints
		for waypoint in msg.points:
			self.waypoints.put(waypoint)  # Add waypoints to the queue

	def goal_callback(self, msg):
		# Process the received message from the "goal" topic
		self.get_logger().info(f'Received goal: {msg}')
	def pose_callback(self, odom_msg):
	# Update the current pose of the robot
		self.update_current_pose(odom_msg.pose.pose.position)

	def move_to_next_waypoint(self):
		if not self.waypoints.empty():
			next_waypoint = self.waypoints.queue[0]
			# print(next_waypoint)
			# Calculate the direction to the next waypoint
			if self.current_pose is not None:
				print(self.current_pose)
				angle = atan2(next_waypoint.y - self.current_pose.y, next_waypoint.x - self.current_pose.x)
				distance = sqrt((next_waypoint.x - self.current_pose.x)**2 + (next_waypoint.y - self.current_pose.y)**2)

				# Move the robot towards the next waypoint (publish Twist message)
				twist_cmd = Twist()
				twist_cmd.linear.x = distance  # Set linear velocity based on the distance
				twist_cmd.angular.z = angle  # Set angular velocity based on the angle
				self.cmd_publisher.publish(twist_cmd)

				# Check if the robot has reached the waypoint
				if distance < 0.1:
					self.waypoints.get()  # Remove the traversed waypoint from the queue
			else:
				self.get_logger().info("waiting odomerty reading(current position)")
	def stop_robot(self):
		# Stop the robot
		stop_cmd = Twist()
		stop_cmd.linear.x = 0.0
		stop_cmd.angular.z = 0.0
		self.cmd_publisher.publish(stop_cmd)
	def update_current_pose(self, pose):
		self.current_pose = pose

