# CMPM 146 P1

# ----- Dustin Halsey ---------
# CruzID: dhalsey

# ----- Austin Kellar ---------
# CruzID: akellar


from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush
from copy import deepcopy

# Helper fuction for dikjstra
# returns the key with lowest distance from source cell
# Args:
#   queue: the input dictionary
#   dist: a dictionary containing the distances from the source cell
def key_of_min_distance(queue, dist):
    min = inf
    key = next(iter(queue)) # first key of the queue
    for k, v in queue.items():
        if dist[k] < min:
            min = dist[k]
            key = k
    return key

# returns true if 2 cells are diagonal from each other, false Otherwise
# Args:
#   start: origin cell
#   end: destination cell
def is_diagonal(start, end):
    start_x, start_y = start
    end_x, end_y = end
    return abs(start_x - end_x) == abs(start_y - end_y)

# Returns the weight of an input cell (number if space, 1 if waypoint)
# Args:
#   level: the level object
#   cell: the cell to calculate the weight of
def weight(level, cell):
    if cell in level['waypoints']:
        return 1
    else:
        return level['spaces'][cell]

# Returns the cost to travel between 2 cells
# Args:
#   level: the level object
#   start: The origin cell
#   end: The destination cell
def cost(level, start, end):
    start_weight = weight(level, start)
    end_weight = weight(level, end)

    if is_diagonal(start, end):
        return (0.5 * sqrt(2) * start_weight) + (0.5 * sqrt(2) * end_weight)
    else:
        return (0.5 * start_weight) + (0.5 * end_weight)

def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """

    x,y = cell
    adj_spaces = []

    for i in range (-1, 2):
        for j in range (-1, 2):
            adj_space = (x + i, y + j)
            if adj_space != cell:
                if adj_space in level['spaces']:
                    edge_cost = cost(level, cell, adj_space)
                    adj_spaces.append((adj_space, edge_cost))


    return adj_spaces

    pass

# Dijkstra's shortest path algorithm
# This implementation finds shortest path to all nodes reachable from initial_position
# We implemented this helper function to reuse some code for the 2 required functions
# Args:
#   initial_position: starting cell
#   graph: the level object
#   adj: function to find neighbor nodes and their costs
#   dist: a reference to a blank dictionary which stores the distances from the source cell
#   parents: a reference to a blank dictionary which stores the parents of all cells in the path from the source cell
def dijkstra(initial_position, graph, adj, dist, parents={}, destination = None):
    if initial_position not in graph['spaces']:
        raise Exception('The initial_position is an invalid position!')

    visited = [initial_position]

    for space in graph['spaces']:
        dist[space] = inf
        parents[space] = None

    dist[initial_position] = 0

    queue = { initial_position: graph['spaces'][initial_position]}

    while len(queue) > 0:
        current_space = key_of_min_distance(queue, dist)

        # if a destination was passed into this helper function
        # and we reached the destination, stop here
        if current_space == destination:
            break;

        queue.pop(current_space, None)

        for neighbor in adj(graph, current_space):
            neighbor_point, cost = neighbor
            if neighbor_point not in visited:
                visited.append(neighbor_point)
                queue[neighbor_point] = cost
            total_dist = dist[current_space] + cost
            if total_dist < dist[neighbor_point]:
                dist[neighbor_point] = total_dist
                parents[neighbor_point] = current_space

def compile_path(parents, source, destination):
    path = []
    current_node = destination

    while current_node != None:
        path.insert(0, current_node)
        current_node = parents[current_node]

    if path[0] == source:
        return path
    else:
        return None

    return path

def dijkstras_shortest_path(initial_position, destination, graph, adj, filename="test_maze.txt"):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """

    if destination not in graph['spaces']:
        raise Exception('The destination is an invalid position!')

    dist = {}
    parents = {}

    dijkstra(initial_position, graph, adj, dist, parents, destination)

    #writes the printed ascii output of shortest path to a file called filename - .txt + _path.txt
    write_level_to_file(graph, compile_path(parents,initial_position,destination), filename)

    return compile_path(parents, initial_position, destination)

    pass


def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """

    dist = {}
    visited = [initial_position]

    dijkstra(initial_position, graph, adj, dist)

    return dist

    pass


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges, filename)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


#writes the printed character shortest path to an output file
#note this is a modified version of the given function show_level()
# This was changed to allow the output to be printed to a file instead of just to console
# Args:
#    level: the level object to be written to file
#    path: a list of cells that represent the shortest path to be drawn
#    filename: the file name of the mazer
#Output:
#    The maze with the shortest path drawn, written to a file called 'filename' - .txt + _path.txt"
def write_level_to_file(level, path=[], filename='test_maze.txt'):
    if filename.endswith('.txt'):
        filename = filename[:-4] #removes the .txt so we can add _path
        filename += '_path.txt' #adds the correct path extension

    xs, ys = zip(*(list(level['spaces'].keys()) + list(level['walls'])))
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = min(ys), max(ys)
    if(path!=None):
        path_cells = set(path)
    else:
        path_cells = []

    chars = []
    inverted_waypoints = {point: char for char, point in level['waypoints'].items()}

    for j in range(y_lo, y_hi + 1):
        for i in range(x_lo, x_hi + 1):

            cell = (i, j)
            if cell in path_cells:
                chars.append('*')
            elif cell in level['walls']:
                chars.append('X')
            elif cell in inverted_waypoints:
                chars.append(inverted_waypoints[cell])
            elif cell in level['spaces']:
                chars.append(str(int(level['spaces'][cell])))
            else:
                chars.append(' ')

        chars.append('\n')

    assert '.txt' in filename, 'Error: filename does not contain file type of ".txt".'
    f = open(filename, "w")
    if (path_cells == []): #if there is no path, print an error in the path file
        f.write("No path possible!\n")
    f.write(''.join(chars))


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]

    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'test_maze.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_maze_costs.csv')
