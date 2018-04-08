from enum import Enum
from queue import PriorityQueue
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
from tqdm import tqdm
from udacidrone.frame_utils import global_to_local

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import os
import re


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


class Obstacle:
    def __init__(self, corners, height):
        self._polygon = Polygon(corners)
        self._height = height

    @property
    def centroid(self):
        return self._polygon.centroid.x, self._polygon.centroid.y

    @property
    def height(self):
        return self._height

    def contains(self, point):
        return point[2] <= self._height and self._polygon.contains(Point(point[0:2]))

    def crosses(self, line):
        return self._polygon.crosses(line)


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if (x - 1 < 0 and y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 and y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n and y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n and y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star_grid(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def a_star_graph(graph, heuristic, start, goal):
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            found = True
            break

        else:
            for next_node in graph[current_node]:
                # get the tuple representation
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node)
    path = []
    path_cost = np.inf
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
        print('Found a path! The cost is {}'.format(path_cost))
    else:
        print('Failed to find a path!')

    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position), ord=2)


def get_global_home(map_name, alt0=0.0):
    with open(map_name, 'r') as file:
        lat0, lon0 = re.findall(r'-?[1-9]\d*\.?\d*', file.readline())

    global_home = [float(lon0), float(lat0), alt0]
    return global_home


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


def create_voxmap(data, voxel_size=30):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.

    The `voxel_size` argument sets the resolution of the voxel map.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for datum in data:
        x, y, z, dx, dy, dz = datum.astype(np.int32)
        obstacle = np.array(((x - dx, x + dx),
                             (y - dy, y + dy),
                             (z - dz, z + dz)))
        obstacle[0] = (obstacle[0] - north_min) // voxel_size
        obstacle[1] = (obstacle[1] - east_min) // voxel_size
        obstacle[2] = obstacle[2] // voxel_size
        voxmap[obstacle[0][0]:obstacle[0][1], obstacle[1][0]:obstacle[1][1], obstacle[2][0]:obstacle[2][1]] = True

    return voxmap


def create_graph(polygons, nodes, k=5):
    g = nx.Graph()
    tree = KDTree(nodes, metric='euclidean')
    for n1 in tqdm(nodes):
        idxs = tree.query(n1.reshape(1, -1), k=k, return_distance=False)
        for n2 in nodes[idxs[0][1:]]:
            if can_connect(polygons, n1, n2):
                dist = np.linalg.norm(n2 - n1, ord=2)
                g.add_edge(tuple(n1), tuple(n2), weight=dist)
    return g


def extract_polygons(data, safety_distance=5):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        north_min = north - d_north - safety_distance
        north_max = north + d_north + safety_distance
        east_min = east - d_east - safety_distance
        east_max = east + d_east + safety_distance
        corners = [(north_min, east_min),
                   (north_min, east_max),
                   (north_max, east_max),
                   (north_max, east_min)]

        # Compute the height of the polygon
        height = alt + d_alt + safety_distance

        # Once you've defined corners, define polygons
        polygons.append(Obstacle(corners, height))

    return polygons


def random_sample(data, polygons, num_samples=300, zmin=2, zmax=10):
    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])
    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])

    xvals = np.random.uniform(xmin, xmax, num_samples)
    yvals = np.random.uniform(ymin, ymax, num_samples)
    zvals = np.random.uniform(zmin, zmax, num_samples)

    samples = np.array((xvals, yvals, zvals)).T
    polygon_centroids = np.array([p.centroid for p in polygons])
    tree = KDTree(polygon_centroids, metric='euclidean')

    valid_samples = []
    while len(valid_samples) < num_samples:
        x = np.random.randint(xmin, xmax)
        y = np.random.randint(ymin, ymax)
        z = np.random.randint(zmin, zmax)
        point = np.array((x, y, z))
        _, idx = tree.query(point[0:2].reshape(1, -1))
        if not polygons[int(idx)].contains(point):
            valid_samples.append(point)

    return np.array(valid_samples)


def can_connect(polygons, n1, n2):
    line = LineString((n1[0:2], n2[0:2]))
    for poly in polygons:
        if min(n1[2], n2[2]) < poly.height and poly.crosses(line):
            return False
    return True


def closest_node(graph, point):
    nodes = np.array(graph.nodes)[:, 0:2]
    _, idx = KDTree(nodes, metric='euclidean').query(np.array(point[0:2]).reshape(1, -1))
    return list(graph.nodes)[int(idx)]


def probabilistic_roadmap(data, polygons, start_position=None, goal_position=None, zmax=20,
                          num_samples=300, k=10, safety_distance=5):

    if goal_position is None:
        goal_position = (0, 0, 5)

    grid, north_offset, east_offset = create_grid(data, goal_position[2], safety_distance)

    if not os.path.exists('graph.gpickle'):
        nodes = random_sample(data, polygons, zmax=zmax, num_samples=num_samples)
        graph = create_graph(polygons, nodes, k=k)
        nx.write_gpickle(graph, 'graph.gpickle')
        plot_graph(data, grid, graph)
    else:
        graph = nx.read_gpickle('graph.gpickle')

    if start_position is not None and goal_position is not None:
        start_node = closest_node(graph, start_position)
        goal_node = closest_node(graph, goal_position)
        print('Start node: {}'.format(start_node))
        print('Goal node:  {}'.format(goal_node))

        # Run A* on the graph
        path, cost = a_star_graph(graph, heuristic, start_node, goal_node)

        # Plot the graph
        plot_graph(data, grid, graph, start_position, goal_position, north_offset, east_offset, path,
                   graph_name='graph_{}_{}'.format(start_node, goal_node))

        return path, cost


def construct_graph(map_name, num_samples=1000, k=10, zmax=20):
    print('Constructing graph ...')
    if not os.path.exists('graph.gpickle'):
        data = np.loadtxt(map_name, delimiter=',', dtype='Float64', skiprows=2)
        polygons = extract_polygons(data)
        probabilistic_roadmap(data, polygons, num_samples=num_samples, k=k, zmax=zmax)
    else:
        print('Graph exists!')


def prune_path(polygons, path):
    print('Pruning the path ...')

    pruned_path = [p for p in path]

    # prune the path!
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p3 = pruned_path[i + 2]
        if can_connect(polygons, p1, p3):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path


def plot_map_2D(grid, start_position=None, goal_position=None,
                north_offset=None, east_offset=None, path=[],
                show_plot=False, plot_name='grid'):

    plt.figure(figsize=(8, 8))

    print('Plotting 2D map ...')
    plt.imshow(grid, cmap='Greys', origin='lower')

    # Plot start and goal
    if start_position is not None:
        plt.plot(start_position[1] - east_offset, start_position[0] - north_offset, 'r^', markersize=8)
    if goal_position is not None:
        plt.plot(goal_position[1] - east_offset, goal_position[0] - north_offset, 'r*', markersize=15)

    # Plot the path
    if len(path):
        path_2d = np.array(path)[:, 0:2] - np.array((north_offset, east_offset))
        plt.plot(path_2d[:, 1], path_2d[:, 0], 'g')
        plt.scatter(path_2d[:, 1], path_2d[:, 0])

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.savefig('Logs/{}'.format(plot_name))
    if show_plot:
        plt.show()
    plt.close()


def plot_map_3D(voxmap, voxel_size=30, add_height=100):
    print('Plotting 3D map ...')
    fig = plt.figure(figsize=(12, 12))
    ax = fig.gca(projection='3d')
    ax.voxels(voxmap, edgecolor='k')
    ax.set_xlim(voxmap.shape[0], 0)
    ax.set_ylim(0, voxmap.shape[1])
    # add height so the buildings aren't so tall
    ax.set_zlim(0, voxmap.shape[2] + add_height // voxel_size)

    plt.xlabel('North')
    plt.ylabel('East')

    plt.show()


def plot_graph(data, grid, graph, start_position=None, goal_position=None,
               north_offset=None, east_offset=None,
               path=[], graph_name='graph'):

    print('Plotting graph ...')
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='Greys', origin='lower')

    if start_position is None and goal_position is None:
        # Draw edges
        for (n1, n2) in graph.edges:
            plt.plot([n1[1] - east_offset,  n2[1] - east_offset],
                     [n1[0] - north_offset, n2[0] - north_offset],
                     'black', alpha=0.5)

        # Draw connected nodes
        for n1 in graph.nodes:
            plt.scatter(n1[1] - east_offset, n1[0] - north_offset, c='blue')

    # Draw the path
    if len(path):
        path_2d = np.array(path)[:, 0:2] - np.array((north_offset, east_offset))
        plt.plot(path_2d[:, 1], path_2d[:, 0], 'g')
        plt.plot(path_2d[:, 1], path_2d[:, 0], 'o')

    # plot start and goal
    if start_position is not None:
        plt.plot(start_position[1] - east_offset,
                 start_position[0] - north_offset,
                 'r^', markersize=8)
    if goal_position is not None:
        plt.plot(goal_position[1] - east_offset,
                 goal_position[0] - north_offset,
                 'r*', markersize=15)

    plt.xlabel('EAST')
    plt.ylabel('NORTH')

    if not os.path.exists('Logs'):
        os.mkdir('Logs')
    plt.savefig('Logs/{}'.format(graph_name))
    plt.close()


if __name__ == '__main__':

    map_name = 'colliders.csv'
    MAX_ALTITUDE = 10
    SAFETY_DISTANCE = 5
    num_attempts = 1
    num_samples = 100
    k = 5

    global_home = get_global_home(map_name)

    data = np.loadtxt(map_name, delimiter=',', dtype='Float64', skiprows=2)
    polygons = extract_polygons(data, SAFETY_DISTANCE)

    start_global_position = (-122.3973419, 37.792567, 0)
    start_position = global_to_local(start_global_position, global_home)
    goal_position = random_sample(data, polygons, num_samples=1, zmax=MAX_ALTITUDE).ravel()
    print(start_position, goal_position)

    for i in range(num_attempts):
        print('\nAttempt {}:'.format(i+1))
        path, cost = probabilistic_roadmap(data, polygons, start_position, goal_position, MAX_ALTITUDE,
                                           num_samples=int(num_samples*(1+i)), k=k)
        if len(path) != 0:
            break

    print('Done!')