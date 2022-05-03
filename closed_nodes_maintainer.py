"""Closed_Nodes_Maintainer.py - Maintains a list of closed nodes using a more advanced and efficient algorithm."""


import math


class ClosedNodesMaintainer:

    def __init__(self, grid_size, width, height):
        self.GRID_SIZE = grid_size

        self.grid = self._generate_grid(width, height, grid_size)


    def _generate_grid(self, width, height, grid_size):
        grid_cells_width = int(width / grid_size)
        grid_cells_height = int(height / grid_size)

        row = [[] for _ in range(grid_cells_width)]
        grid = [row[:] for _ in range(grid_cells_height)]

        return grid


    def record_new_node(self, node):
        x, y = node.coordinates

        x_grid = int(x / self.GRID_SIZE)
        y_grid = int(y / self.GRID_SIZE)

        self.grid[y_grid][x_grid].append(node)



    def is_there_duplicate(self, node):
        x, y = node.coordinates

        x_grid = int(x / self.GRID_SIZE)
        y_grid = int(y / self.GRID_SIZE)

        relative_grid_squares = [-1, 0, 1]
        buckets_to_search = []
        for dx in relative_grid_squares:
            for dy in relative_grid_squares:
                new_x_grid = x_grid + dx
                new_y_grid = y_grid + dy

                is_x_valid = 0 <= new_x_grid < len(self.grid[0])
                is_y_valid = 0 <= new_y_grid < len(self.grid)
                if is_x_valid and is_y_valid:
                    buckets_to_search.append(self.grid[new_y_grid][new_x_grid])

        for bucket in buckets_to_search:
            for existing_node in bucket:
                if self._is_node_duplicate(node, existing_node):
                    return True

        return False


    def _is_node_duplicate(self, node1, node2):
        x1, y1 = node1.coordinates
        x2, y2 = node2.coordinates

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if math.sqrt(dx**2 + dy**2) > 0.05:
            return False

        dtheta = abs(node1.theta - node2.theta)
        if dtheta >= 10:
            return False

        return True

        # TODO - if two thetas are off by only 30 degrees, they are still considered duplicates
        # Will have to consider cases where one is 330 degrees and the other is 0 degrees
