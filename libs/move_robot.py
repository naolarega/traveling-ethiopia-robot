#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import math

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/wheel/odometry', self.odom_callback, 10) 
        self.odom = None
        # Path callback flag
        #self.path_received = False

    def move_to_goal(self, goal_pose):
        self.get_logger().info('Moving to goal pose: {}'.format(goal_pose))
        x, y, z, qx, qy, qz, qw = goal_pose
        #self.get_logger().info('i am here')
        #while not self.path_received:
            #rclpy.spin_once(self)
        
        while not self.is_pose_reached(goal_pose):
            linear_velocity = self.calculate_linear_velocity(x, y)
            angular_velocity = self.calculate_angular_velocity(qz, qw, qx, qy)
            
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_msg.angular.z = angular_velocity

            self.cmd_vel_pub.publish(cmd_vel_msg)
            rclpy.spin_once(self)

        self.stop_robot()
        #self.path_received = False
        
    def odom_callback(self, msg):
        self.odom = msg
        current_pose = self.odom.pose.pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        self.get_logger().info(f"point: ({current_x},{current_y})")
        
    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def is_pose_reached(self, goal_pose):
        if self.odom is not None:
            current_pose = self.odom.pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            current_distance = math.sqrt((current_x - goal_pose[0])**2 + (current_y - goal_pose[1])**2)
            # Check if the current distance to the goal is within a threshold
            diff_x=current_distance-current_x
            diff_y=current_distance-current_y
            if current_distance  < 0.1:
                return True
        else:
              self.get_logger().info('odom.pose.pose does not exist or is not accessible')

        return False

    def calculate_linear_velocity(self, goal_x, goal_y):
        if self.odom is not None:
            current_pose = self.odom.pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            current_distance = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
            # Check if the current distance to the goal is within a threshold
            if current_distance < 0.1:
                return 0.0  # Stop the robot

            linear_velocity = 0.1 * current_distance  # Example linear velocity scaling
            return linear_velocity

        return 0.0

    def calculate_angular_velocity(self, goal_qz, goal_qw, goal_qx, goal_qy):
        if self.odom is not None:
            current_pose = self.odom.pose.pose
            current_qx = current_pose.orientation.x
            current_qy = current_pose.orientation.y
            current_qz = current_pose.orientation.z
            current_qw = current_pose.orientation.w
            
            

            # Convert the current and goal orientations to Euler angles
            current_euler = self.quaternion_to_euler(current_qx, current_qy, current_qz, current_qw)
            goal_euler = self.quaternion_to_euler(goal_qx, goal_qy, goal_qz, goal_qw)

            # Calculate the difference in yaw angles
            delta_yaw = goal_euler[2] - current_euler[2]

            # Adjust the difference to the range of -pi to pi
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            # Scale the angular velocity based on the difference in yaw angles
            angular_velocity = 0.1 * delta_yaw
            return angular_velocity

        return 0.0
    def quaternion_to_euler(self, qx,qy,qz,qw):
        q0 = qw
        q1 = qx
        q2 = qy
        q3 = qz

        # Calculate roll (x-axis rotation)
        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))

        # Calculate pitch (y-axis rotation)
        sin_pitch = 2 * (q0 * q2 - q3 * q1)
        if abs(sin_pitch) >= 1:
            pitch = math.copysign(math.pi / 2, sin_pitch)
        else:
            pitch = math.asin(sin_pitch)

        # Calculate yaw (z-axis rotation)
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

        return roll, pitch, yaw


