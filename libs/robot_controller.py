import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from libs.msg import Path, Pose2DStamped
from math import atan2, sqrt
from queue import Queue

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.current_pose = Pose2D()
        self.waypoints = Queue()
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription_path = self.create_subscription(
            Path,
            'path_topic',
            self.path_callback,
            10
        )

        self.subscription_pose = self.create_subscription(
            Odometry,
            'pose_topic',
            self.pose_callback,
            10
        )

    def path_callback(self, msg):
        # Process the received path
        self.get_logger().info('Received path: %s' % msg)
        self.waypoints.queue.clear()  # Clear existing waypoints
        for waypoint in msg.waypoints:
            self.waypoints.put(waypoint)  # Add waypoints to the queue

    def pose_callback(self, odom_msg):
        # Update the current pose of the robot
        self.update_current_pose(odom_msg.pose.pose.position)

    def move_to_next_waypoint(self):
        if not self.waypoints.empty():
            next_waypoint = self.waypoints.queue[0]

            # Calculate the direction to the next waypoint
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
            # Stop the robot when the queue is empty
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_publisher.publish(stop_cmd)

    def update_current_pose(self, pose):
        self.current_pose = pose

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()

    # Set the desired rate (e.g., 10 Hz)
    rate = node.create_rate(10)

    while rclpy.ok() and not node.waypoints.empty():
        node.move_to_next_waypoint()
        rclpy.spin_once(node)
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
