#!/usr/bin/env python3
import rclpy
import time

from libs.move_robot import MoveRobot

def perform_navigation_actions(robot, goal_pose):
    # Assuming you have an instance of the MoveRobot class and the goal pose
    robot.move_to_goal(goal_pose)
    
def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()

    # Define the goal poses (point A and point B)
    goal_poses = [(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), (2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0), (4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)]

    # Move to each goal pose
    for goal_pose in goal_poses:
        perform_navigation_actions(move_robot, goal_pose)
        time.sleep(1)
    
    print("Goal poses reached!")

    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
