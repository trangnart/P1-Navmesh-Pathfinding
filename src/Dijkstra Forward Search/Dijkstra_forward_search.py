from maze_environment import load_level, show_level
from math import inf, sqrt
from heapq import heappop, heappush

def bidirectional_a_star(initial_position, destination, graph, adj):
    forward_paths = {initial_position: []}
    forward_pathcosts = {initial_position: 0}
    forward_queue = []
    heappush(forward_queue, (0, initial_position))

    backward_paths = {destination: []}
    backward_pathcosts = {destination: 0}
    backward_queue = []
    heappush(backward_queue, (0, destination))

    meet_pts = None

    while forward_queue and backward_queue:
        forward_priority, forward_cell = heappop(forward_queue)
        backward_priority, backward_cell = heappop(backward_queue)

        # Check if the forward and backward searches meet
        if forward_cell in backward_paths:
            meet_pts = forward_cell
            break

        # investigate children in the forward search
        for (forward_child, step_cost) in adj(graph, forward_cell):
            forward_cost_to_child = forward_priority + transition_cost(graph, forward_cell, forward_child)
            if forward_child not in forward_pathcosts or forward_cost_to_child < forward_pathcosts[forward_child]:
                forward_pathcosts[forward_child] = forward_cost_to_child
                forward_paths[forward_child] = forward_cell
                heappush(forward_queue, (forward_cost_to_child, forward_child))

        # investigate children in the backward search
        for (backward_child, step_cost) in adj(graph, backward_cell):
            backward_cost_to_child = backward_priority + transition_cost(graph, backward_cell, backward_child)
            if backward_child not in backward_pathcosts or backward_cost_to_child < backward_pathcosts[backward_child]:
                backward_pathcosts[backward_child] = backward_cost_to_child
                backward_paths[backward_child] = backward_cell
                heappush(backward_queue, (backward_cost_to_child, backward_child))

    if meet_pts is not None:
        forward_path = path_to_cell(meet_pts, forward_paths)
        backward_path = path_to_cell(meet_pts, backward_paths)[::-1]  # Reverse the backward path
        return forward_path + backward_path[1:]  # Exclude the meet_pts for the final path
    else:
        return None

def path_to_cell(cell, paths):
    if cell == []:
        return []
    return path_to_cell(paths[cell], paths) + [cell]


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
    res = []
    for delta in [(x, y) for x in [-1,0,1] for y in [-1,0,1] if not (x==0 and y==0)]:
        new = (cell[0] + delta[0], cell[1] + delta[1])
        if new in level['spaces']:
            res.append((new, transition_cost(level, new, cell)))
    return res

def transition_cost(level, cell, cell2):
    distance = sqrt((cell2[0] - cell[0])**2 + (cell2[1] - cell[1])**2)
    average_cost = (level['spaces'][cell] + level['spaces'][cell2])/2
    return distance * average_cost


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
    path = bidirectional_a_star(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'example.txt', 'a','e'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)
