
import os
from .city_graph import City,Point
class CityFileReader:
    @staticmethod
    def read_cities_and_distances(file_name, city_graph):
        cities = {}
         # Get the current directory
        file_path = f'state/{file_name}'
        #print(file_path)
        # File path relative to the current directory
        #file_path = os.path.join(current_dir, '/install/two_wheeled_robot/share/two_wheeled_robot/data', file_name)
        #print(file_path)
        with open(file_path, 'r') as file:
                  for line in file:
                    parts = line.strip().split(',')
                    if len(parts)==3:
                        city_name,x,y = parts
                        cities[city_name] = City(city_name,Point(int(x),int(y)))
                    elif len(parts) == 2:
                        source, destination = parts[0], parts[1]
                        city_graph.add_distance(cities[source], cities[destination])
        return cities
    
