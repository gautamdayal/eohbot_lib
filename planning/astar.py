import networkx as nx 
import numpy as np
import matplotlib.pyplot as plt

# Heuristic function (euclidean distance works)
def euclidean_dist(p1, p2):
    return abs(p1[1]-p2[1]) + abs(p1[0] - p2[0])

# Generate binary occupancy grid using obstacle locations. Process the robot position to be a grid square as well
def generate_occupancy(robot_position, obstacle_positions):
    # Create blank environment with occupied borders to handle edge cases
    GRANULARITY = 250 # Change this to get true size of robot as a granularity factor for the grid
    shape = (14, 12)
    result = np.zeros(shape)
    result[0] = 1
    result[:,0] = 1
    result[:,shape[1] - 1] = 1
    result[shape[0] - 1] = 1
    
    # Add obstacles based on given positions
    robot_position_grid = np.around(robot_position/GRANULARITY) + 1 # add one to account for padding around the environment
    obstacle_positions_grid = [np.around(pos/GRANULARITY) for pos in obstacle_positions]
    # print(obstacle_positions_grid)
    for obstacle in obstacle_positions_grid:
        result[min(int(obstacle[0] + 1), 13), min(int(obstacle[1] + 1), 11)] = 1

    # return grid and start point
    return result, (int(robot_position_grid[0]), int(robot_position_grid[1]))


# Turn occupancy into a searchable networkx graph
def get_graph(occupancy_grid):
    G = nx.Graph()
    rows, cols = occupancy_grid.shape
    for i in range(rows):
        for j in range(cols):
            if occupancy_grid[i, j] == 0:  
                G.add_node((i, j))
                if i > 0 and occupancy_grid[i - 1, j] == 0:
                    G.add_edge((i, j), (i - 1, j))
                if i < rows - 1 and occupancy_grid[i + 1, j] == 0:
                    G.add_edge((i, j), (i + 1, j))
                if j > 0 and occupancy_grid[i, j - 1] == 0:
                    G.add_edge((i, j), (i, j - 1))
                if j < cols - 1 and occupancy_grid[i, j + 1] == 0:
                    G.add_edge((i, j), (i, j + 1))
    return G


# Get path to follow from astar
def get_path(G, start_node, goal_node):
    path = nx.astar_path(G, start_node, goal_node, euclidean_dist)
    return path

# Get only waypoints from the path instead of every point
def get_waypoints(path):
    result = []
    result.append(path[0])
    for i in range(1, len(path)-1):
        prev = path[i - 1]
        curr = path[i]
        next = path[i + 1]
        slope_1 = (curr[1] - prev[1])/(curr[0] - prev[0] + 0.1)
        slope_2 = (next[1] - curr[1])/(next[0] - curr[0] + 0.1)
        if slope_1 != slope_2:
            result.append(curr)
    result.append(path[len(path) - 1])
    return result

