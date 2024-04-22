import networkx as nx 
import numpy as np
import matplotlib.pyplot as plt

# Heuristic function (euclidean distance works)
def mahhattan_dist(p1, p2):
    return abs(p1[1]-p2[1]) + abs(p1[0] - p2[0])

def euclidean(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

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
    # positions = [[0, 0], [0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]
    positions=[[0, 0], [1, 0], [-1, 0]]
    for obstacle in obstacle_positions_grid:
        x = obstacle[0] + 1
        y = obstacle[1] + 1
        for x_offset, y_offset in positions:
            result_x = x + x_offset
            result_y = y + y_offset
            result_x = int(max(0, min(result_x, 13)))
            result_y = int(max(0, min(result_y, 11)))
            # print(f'x: {result_x} y: {result_y}')
            result[result_x, result_y] = 1

    # return grid and start point
    # print(result)
    return result, (int(robot_position_grid[0]), int(robot_position_grid[1]))


# Constant that defines the edge weight of going into or out of occupied space
OBSTACLE_PENALTY = 10

# Turn occupancy into a searchable networkx graph
def get_graph(occupancy_grid):
    G = nx.Graph()
    rows, cols = occupancy_grid.shape
    for row in range(rows):
        for col in range(cols):
            
            G.add_node((row, col))
            if row > 0 and occupancy_grid[row - 1, col] == 0:
                G.add_edge((row, col), (row - 1, col))
            if row < rows - 1 and occupancy_grid[row + 1, col] == 0:
                G.add_edge((row, col), (row + 1, col))
            if col > 0 and occupancy_grid[row, col - 1] == 0:
                G.add_edge((row, col), (row, col - 1))
            if col < cols - 1 and occupancy_grid[row, col + 1] == 0:
                G.add_edge((row, col), (row, col + 1))
            # 8 directions
            if row > 0 and col > 0 and occupancy_grid[row-1, col-1] == 0:
                G.add_edge((row, col), (row - 1, col - 1))
            if row < rows-1 and col < cols - 1 and occupancy_grid[row+1, col+1] == 0:
                G.add_edge((row, col), (row + 1, col + 1))
            if row < rows-1 and col > 0 and occupancy_grid[row+1, col-1] == 0:
                G.add_edge((row, col), (row + 1, col - 1))
            if row > 0 and col < cols-1 and occupancy_grid[row-1, col+1] == 0:
                G.add_edge((row, col), (row - 1, col + 1))
    return G


# Get path to follow from astar
def get_path(G, start_node, goal_node):
    path = nx.astar_path(G, start_node, goal_node, mahhattan_dist)
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

