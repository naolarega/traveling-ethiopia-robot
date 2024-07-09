from queue import Queue,PriorityQueue
from collections import defaultdict
class CitySearch:
    # breadth first search
    @staticmethod
    def bfs_shortest_path(graph, start_city, goal_city):
        visited = set()
        queue = Queue()
        queue.put((start_city, [start_city]))

        while not queue.empty():
            current_city, path = queue.get()

            if current_city == goal_city:
                return path

            visited.add(current_city)

            for neighbor, _ in graph.get_neighbors(current_city):
                if neighbor not in visited:
                    queue.put((neighbor, path + [neighbor]))

        return None  # If no path is found
    # depth first search
    @staticmethod
    def dfs_shortest_path(graph, start_city, goal_city):
        visited = set()
        stack = [(start_city, [start_city])]

        while stack:
            current_city, path = stack.pop()

            if current_city.get_city_name() == goal_city.get_city_name():
                return path

            visited.add(current_city.get_city_name())

            for neighbor, _ in graph.get_neighbors(current_city):
                neighbor_name = neighbor.get_city_name()
                if neighbor_name not in visited:
                    stack.append((neighbor, path + [neighbor]))

        return None
   