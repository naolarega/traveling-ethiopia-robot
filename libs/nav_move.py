#!/usr/bin/env python3
import rclpy
import math
import time 

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from math import atan2, asin, copysign,sqrt,pi
from .city_graph import Graph,City,Point as CPoint
from .city_file_reader import CityFileReader
from .city_search import CitySearch

class NavMove(Node):
    def __init__(self):
        super().__init__('speed_controller')
        self.subscription = self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.path=self.user_input()
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = self.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    def euler_from_quaternion(self, quat):
        # Convert a Quaternion message to Euler angles
        roll, pitch, yaw = 0.0, 0.0, 0.0

        qx = quat[0]
        qy = quat[1]
        qz = quat[2]
        qw = quat[3]

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = copysign(math.pi / 2.0, sinp)
        else:
            pitch = asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    def run(self):  
        # print(self.path)
        if self.path is not None:
            city_name_dict={}
            for city in self.path:
                point=str(float(city.get_point().x))+" "+str(float(city.get_point().y))
                city_name_dict[point]=city
            points = [city.get_point() for city in self.path]
            self.goals= [Point(x=float(p.x), y=float(p.y), z=0.0) for p in points]
            self.speed = Twist()
            current_goal_index = 0
            start_city=self.goals[current_goal_index]
            print("starting  from city "+city_name_dict[str(start_city.x)+" "+str(start_city.y)].get_city_name())
            # while current_goal_index < len(self.goals):
            #         goal = self.goals[current_goal_index]    
            #         inc_x = goal.x - self.x
            #         inc_y = goal.y - self.y
            #         distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)
            #         # print("distance"+str(distance_to_goal))
            #         angle_to_goal = atan2(inc_y, inc_x)
            #         # print("angle"+str(angle_to_goal))
            #         angle_diff = angle_to_goal - self.theta
            #         if abs(angle_diff) > 0.2:
            #             # If the angle difference is significant, rotate towards the goal
            #             self.speed.linear.x = 0.0
            #             if angle_diff > 0:
            #                 self.speed.angular.z = -0.2  # Turn counterclockwise
            #             else:
            #                 self.speed.angular.z = 0.2  # Turn clockwise
            #         else:
            #             # If the robot is oriented towards the goal, check the distance
            #               # Stop rotation and move forward towards the goal
            #             self.speed.angular.z = 0.0
            #             if distance_to_goal < 0.2:
            #                 # If the robot is close enough to the goal, stop and move to the next goal
            #                 self.speed.linear.x = 0.0
            #                 # self.speed.angular.z = 0.0
            #                 if 0 < current_goal_index < len(self.goals) - 1:
            #                     print("City " + city_name_dict[str(goal.x) + " " + str(goal.y)].get_city_name() + " reached")  
            #                 current_goal_index += 1  # Move to the next goal
            #             else:
            #                 # Move forward towards the goal
            #                 self.speed.linear.x = 0.5
            #                 # self.speed.angular.z = 0.0

            #         self.publisher.publish(self.speed)
            #         rclpy.spin_once(self)         
            # self.speed.linear.x = 0.0
            # self.speed.angular.z = 0.0
            # self.publisher.publish(self.speed) 
            while current_goal_index < len(self.goals):
                    goal = self.goals[current_goal_index]    
                    inc_x = goal.x - self.x
                    inc_y = goal.y - self.y
                    distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)
                    # print(distance_to_goal)
                    angle_to_goal = atan2(inc_y, inc_x)
                    # print(angle_to_goal)
                    if distance_to_goal < 0.2:
                        self.speed.linear.x = 0.0
                        self.speed.angular.z = 0.0
                        if 0 < current_goal_index < len(self.goals) - 1:
                            print("City " + city_name_dict[str(goal.x) + " " + str(goal.y)].get_city_name() + " reached")  
                        current_goal_index += 1  # Move to the next goal
                    else:
                        angle_diff = angle_to_goal - self.theta
                        if abs(angle_diff) > 0.2:
                            self.speed.linear.x = 0.0
                            self.speed.angular.z = copysign(0.3, angle_diff)
                        else:
                            self.speed.linear.x = 0.5
                            self.speed.angular.z = 0.0

                    self.publisher.publish(self.speed)
                    rclpy.spin_once(self)        
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.publisher.publish(self.speed) 
            goal_city=self.goals[len(self.goals)-1]
            print("goal city "+city_name_dict[str(goal_city.x)+" "+str(goal_city.y)].get_city_name()+" reached")
        else:
            self.get_logger().info("No path found. Please select cities and algorithm from given prompt.")
       
            # self.move_to_waypoints(self.goals)   
    def user_input(self):
        path=None
        cities={}
        graph,city_dic=self.get_cities()
        index=1
        # counter=0
        city_names=""
        for city_name in city_dic.keys():
            city_names=city_names+str(index)+" "+city_name+","
            cities[str(index)]=city_name
            index+=1
        print(city_names)
        start_city_number = input("Please select start city(1-41): ")
        goal_city_number = input("Please select goal city(1-41): ")
        algorithm = input("choose algorithm (bfs,dfs):")
        if start_city_number not in cities or goal_city_number not in cities:
            self.get_logger().info("Invalid city names. Please provide numbers from prompt.")
        else:
             start_city_name=cities[start_city_number]
             goal_city_name=cities[goal_city_number]
             start_city=city_dic[start_city_name]
             goal_city=city_dic[goal_city_name]
             if algorithm not in ['bfs','dfs']:
                self.get_logger().info("Invalid algorithm names. Please provide either bfs or dfs.")
             else:
               path=self.get_paths(start_city,goal_city,algorithm,graph) 
        return path  
    def get_paths(self,start_city,goal_city,algorithm,graph):
        # Ensure the provided cities exist in the graph
        path=None
        self.goal=goal_city.get_point()
        if algorithm=='bfs':
            path=CitySearch.bfs_shortest_path(graph,start_city,goal_city)
            if path is not None:
                path_strings = [city.get_city_name() for city in path]
                self.get_logger().info("BFS Shortest Path:"+ " -> ".join(path_strings))
            else:
                self.get_logger().info("No path found.")
        else:
            path=CitySearch.dfs_shortest_path(graph,start_city,goal_city)
            if path is not None:
                path_strings = [city.get_city_name() for city in path]
                self.get_logger().info("DFS Shortest Path:"+ " -> ".join(path_strings))
            else:
                self.get_logger().info("No path found.")
        return path  
    def get_cities(self):
        graph = Graph()
        cities_dict = CityFileReader.read_cities_and_distances('ethiopia.txt',graph)
        return graph,cities_dict
    def move_between_points(self, end_point):
        # Calculate the vector from the current position to the end point
        inc_x = end_point.x - self.x
        inc_y = end_point.y - self.y

        # Calculate the distance between the current position and the end point
        distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)

        # Calculate the angle between the current position and the end point
        angle_to_goal = atan2(inc_y, inc_x)
        if distance_to_goal < 0.2:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            return True
        else:
            angle_diff = angle_to_goal - self.theta
            if abs(angle_diff) > 0.2:
                self.speed.linear.x = 0.0
                self.speed.angular.z = copysign(0.3, angle_diff)
            else:
                self.speed.linear.x = 0.5
                self.speed.angular.z = 0.0

        # Publish the velocity command and spin once
        self.publisher.publish(self.speed)
        rclpy.spin_once(self)
        
        return False  # Return False if the end point has not been reached yet
    def move_to_waypoints(self, waypoints, stop_duration=1):
        for i, waypoint in enumerate(waypoints):
            # Move towards the current waypoint
            while not self.move_between_points(waypoint):
                pass  # Keep moving until the waypoint is reached

            # Stop at the waypoint for the specified duration
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.publisher.publish(self.speed)
            rclpy.spin_once(self)
            time.sleep(stop_duration)  # Stop for the specified duration
       
