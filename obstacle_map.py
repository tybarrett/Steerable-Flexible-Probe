"""obstacle_map.py - Describes a map of the obstacles in the problem area."""


import math


IN_MARGIN = 1
IN_OBSTACLE = 2


class Polygon:
    def __init__(self, line_segments):
        self.line_segments = line_segments


    def is_point_inside(self, coord, margin):
        x, y = coord

        max_y = max(*[p[1] for p, _, _ in self.line_segments])
        min_y = min(*[p[1] for p, _, _ in self.line_segments])

        if y > max_y or y < min_y:
            return False

        for line_segment in self.line_segments:
            p1, p2, is_region_above = line_segment

            slope_reciprocal = (p1[0] - p2[0]) / (p1[1] - p2[1])
            x_intercept = p1[0] - p1[1] * slope_reciprocal

            if x < y * slope_reciprocal + x_intercept:
                point_on_outside_of_line = is_region_above
            else:
                point_on_outside_of_line = not is_region_above

            point_within_y_window = (y <= max(p1[1], p2[1])) and (y >= min(p1[1], p2[1]))

            # If this point is not on the "obstacle" side of the line (and is on the same row as the line) it is outside
            if point_on_outside_of_line and point_within_y_window:
                return False

        # We went through all of the line_segments, and we are inside each one.
        return IN_OBSTACLE


    def is_point_within_margin(self, coord, margin):

        for line_segment in self.line_segments:
            p1, p2, _ = line_segment

            # Find the distance from the point to the line
            closest_point_on_line = self._get_closest_point_on_line(p1, p2, coord)
            higher_point = p1 if p1[1] > p2[1] else p2
            lower_point = p1 if p1[1] < p2[1] else p2
            if closest_point_on_line[1] > higher_point[1]:
                closest_legal_point = higher_point
            elif closest_point_on_line[1] < lower_point[1]:
                closest_legal_point = lower_point
            else:
                closest_legal_point = closest_point_on_line
            distance_from_point_to_line = math.sqrt(
                (coord[0] - closest_legal_point[0]) ** 2 + (coord[1] - closest_legal_point[1]) ** 2)
            inside_margin = distance_from_point_to_line < margin

            if inside_margin:
                return IN_MARGIN

        return False

    def _get_closest_point_on_line(self, p1, p2, coord):
        p1_to_coord = (coord[0] - p1[0], coord[1] - p1[1])
        p1_to_p2 = (p2[0] - p1[0], p2[1] - p1[1])

        line_length_squared = p1_to_p2[0]**2 + p1_to_p2[1]**2

        # Essentially just the dot product between p1_to_coord and p1_to_p2
        closest_point_proportion = (p1_to_coord[0]*p1_to_p2[0] + p1_to_p2[1]*p1_to_coord[1]) / line_length_squared

        newx = p1[0] + p1_to_p2[0] * closest_point_proportion
        newy = p1[1] + p1_to_p2[1] * closest_point_proportion

        return newx, newy


# This class is specific to Project 2
class ObstacleMap(object):
    def __init__(self, width, height, obstacles, margin):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.clearance_margin = margin

    def is_coordinate_occupied(self, coordinate):
        x, y = coordinate
        # if x < self.clearance_margin or x > self.width - self.clearance_margin:
        #     return IN_MARGIN
        if y < self.clearance_margin or y > self.height - self.clearance_margin:
            return IN_MARGIN

        for obstacle in self.obstacles:
            if obstacle[0] == "POLYGON":
                poly = obstacle[1]
                is_inside = poly.is_point_inside(coordinate, self.clearance_margin)
                if is_inside:
                    return IN_OBSTACLE
                else:
                    is_in_margin = poly.is_point_within_margin(coordinate, self.clearance_margin)
                    if is_in_margin:
                        return IN_MARGIN

            elif obstacle[0] == "CIRCLE":
                _, center, radius = obstacle
                dist_from_center = math.sqrt((x - center[0])**2 + (y - center[1])**2)
                if dist_from_center <= radius:
                    return IN_OBSTACLE
                elif dist_from_center <= radius + self.clearance_margin:
                    return IN_MARGIN

            elif obstacle[0] == "RECTANGLE":
                _, x0, x1, y0, y1 = obstacle
                if x0 <= x <= x1 and y0 <= y <= y1:
                    return IN_OBSTACLE

                # Check with an inflated version of the obstacle for presence in the margin.
                # First, just inflate the edges by CLEARANCE_MARGIN
                if (x0 - self.clearance_margin) <= x <= (x1 + self.clearance_margin):
                    if y0 <= y <= y1:
                        return IN_MARGIN
                if (y0 - self.clearance_margin) <= y <= (y1 + self.clearance_margin):
                    if x0 <= x <= x1:
                        return IN_MARGIN

                # Then, check against the corners.
                bl = (x0, y0)
                tl = (x0, y1)
                br = (x1, y0)
                tr = (x1, y1)
                corners = [bl, tl, br, tr]
                for corner in corners:
                    distance = math.sqrt((x - corner[0])**2 + (y - corner[1])**2)
                    if distance <= self.clearance_margin:
                        return IN_MARGIN

        # We have proven that this point does not exist in any obstacle.
        return False


OBSTACLE_SHAPES = [("RECTANGLE", 0.925, 1.075, 0.425, 0.575),
                   ("RECTANGLE", 1.425, 1.575, 0.2, 0.35),
                   ("RECTANGLE", 1.425, 1.575, 0.65, 0.8),
                   ("RECTANGLE", 1.925, 2.075, 0.425, 0.575)]


def generate_obstacle_map(margin):
    return ObstacleMap(3, 1, OBSTACLE_SHAPES, margin)


if __name__ == "__main__":
    map = generate_obstacle_map(0)
    result = map.is_coordinate_occupied((1, 9))
    print("Is coordinate occupied?")
    print(result)
