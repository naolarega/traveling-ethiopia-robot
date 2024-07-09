from queue import Queue
from collections import defaultdict

class Point:
    def __init__(self, x, y):
        self.x=x
        self.y=y
    def __str__(self):
        return f"point: ({self.x},{self.y})"

class City:
    def __init__(self, city_name, point):
        self.city_name=city_name
        self.point=point
    def get_city_name(self):
        return self.city_name
    def get_point(self):
    	return self.point
    def __str__(self):
        return f"{self.city_name}, point: ({self.point.x},{self.point.y})"

class Graph:
    def __init__(self):
        self.graph = {}

    def add_city(self, city):
        if city not in self.graph:
            self.graph[city] = []
   
    def add_distance(self, source, destination, distance=None):
        self.add_city(source)
        self.add_city(destination)
        if distance is not None:
            # Assuming distances are bidirectional
            self.graph[source].append((destination, distance))
            self.graph[destination].append((source, distance))
        else:
          self.graph[source].append((destination,None))
          self.graph[destination].append((source,None))  

    def get_neighbors(self, city):
        return self.graph.get(city, [])
    
    def __str__(self):
        result = []
        for city, neighbors in self.graph.items():
            neighbor_str = ', '.join([f"{neighbor[0].city_name} ({neighbor[1]})" for neighbor in neighbors])
            result.append(f"{city.city_name} -> Neighbors: {neighbor_str}")
        return '\n'.join(result)

