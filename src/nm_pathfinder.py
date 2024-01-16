import heapq

def heuristic(point1, point2):
    """
    Euclidean distance heuristic for A* algorithm
    """
    x1, y1 = point1
    x2, y2 = point2
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def find_path(source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh using A* algorithm

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
        A path (list of points) from source_point to destination_point if it exists
        A list of boxes explored by the algorithm
    """

    def reconstruct_path(came_from, current):
        """
        Reconstructs the path from the came_from dictionary
        """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.insert(0, current)
        return total_path

    def A_star(start, goal, heuristic_func):
        open_set = [(0, start)]  # Priority queue with initial cost
        came_from = {}

        g_score = {start: 0}

        while open_set:
            current_cost, current_node = heapq.heappop(open_set)

            if current_node == goal:
                return reconstruct_path(came_from, goal)

            for neighbor in mesh.get(current_node, []):
                tentative_g_score = g_score[current_node] + heuristic_func(current_node, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    heapq.heappush(open_set, (tentative_g_score + heuristic_func(neighbor, goal), neighbor))
                    came_from[neighbor] = current_node

        return None

    # Perform A* search from source to destination
    forward_path = A_star(source_point, destination_point, heuristic)

    # If the forward path exists, perform A* search from destination to source
    if forward_path:
        backward_path = A_star(destination_point, source_point, heuristic)
    else:
        backward_path = None

    if backward_path:
        # Combine the forward and backward paths
        combined_path = forward_path[:-1] + backward_path[::-1]
        explored_boxes = set(forward_path) | set(backward_path)
    else:
        combined_path = []
        explored_boxes = set()

    return combined_path, explored_boxes
