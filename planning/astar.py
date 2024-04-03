import networkx as nx 
import numpy as np
import matplotlib.pyplot as plt

# Heuristic function (euclidean distance works)
def euclidean_dist(p1, p2):
    return np.linalg.norm(np.array(p1)-np.array(p2), 2)

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
def get_path(G):
    path = nx.astar_path(G, (1, 1), (8, 8), euclidean_dist)
    return path


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

occupancy_grid = np.array([
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1], 
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
])


path = get_path(get_graph(occupancy_grid))
plt.imshow(occupancy_grid)
for p in get_waypoints(path):
    plt.scatter(p[1], p[0], color="red")
plt.show()

