#!/usr/bin/env python3
import rclpy

from libs.msg import Path, Goal
from geometry_msgs.msg import Point
from rclpy.node import Node
from .city_graph import Graph,City,Point as CPoint
from .city_file_reader import CityFileReader
from .city_search import CitySearch

class SearchNode(Node):
	def __init__(self):
		super().__init__("path_publisher_node")
		self.get_logger().info("publishing path")
		self.path_publisher_ = self.create_publisher(Path, 'path_topic', 10)
		self.goal_publisher_ = self.create_publisher(Goal, 'goal_topic', 10)
		# Prompt the user for input
		self.start_city_name = input("Enter initial city:")
		self.goal_city_name = input("Enter goal city: ")
		self.algorithm = input("choose algorithm (bfs,dfs):")
		self.goal=None
		self.path=self.run() 
		path_strings = [str(city.get_point()) for city in self.path]
		self.get_logger().info("path: " + " -> ".join(path_strings))
	def publish_path(self):
		path_msg = Path()
		goal_msg = Goal()
		points = [city.get_point() for city in self.path]
		path_msg.points = [Point(x=float(p.x), y=float(p.y), z=0.0) for p in points]
		goal_msg.goal = Point(x=float(self.goal.x), y=float(self.goal.y), z=0.0)

		self.path_publisher_.publish(path_msg)
		self.goal_publisher_.publish(goal_msg)

		self.get_logger().info('Published path message with %d points' % len(path_msg.points))
		self.get_logger().info('Published goal message: (%f, %f, %f)' % (goal_msg.goal.x, goal_msg.goal.y, goal_msg.goal.z))
	def run(self):
		graph = Graph()

		cities_dict = CityFileReader.read_cities_and_distances('ethiopia.txt',graph)
		# Ensure the provided cities exist in the graph
		if self.start_city_name not in cities_dict or self.goal_city_name not in cities_dict:
			self.get_logger().info("Invalid city names. Please provide valid city names.")
		else:
			start_city = cities_dict[self.start_city_name]
			goal_city = cities_dict[self.goal_city_name]
			self.goal=goal_city.get_point()
			if self.algorithm not in ['bfs','dfs']:
				self.get_logger().info("Invalid algorithm names. Please provide either bfs or dfs.")
			else:
				if self.algorithm=='bfs':
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
