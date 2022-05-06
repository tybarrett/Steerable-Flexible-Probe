import logging
import random
import math

import closed_nodes_maintainer


GOAL_THRESHOLD = 0.15
MIN_CURVATURE_RADIUS = 0.5 # TODO - choose a realistic value


class Rg_Rrt(object):
    def __init__(self, obstacle_map, length_weight, clearance_weight, risk_weight):
        self.obstacle_map = obstacle_map
        self.open = []
        self.closed = closed_nodes_maintainer.ClosedNodesMaintainer(0.1, self.obstacle_map.width,
                                                                    self.obstacle_map.height)

        # self.traverser = diff_drive_traverser.DiffDriveTraverser(obstacle_map, goal_coordinate)

        logging.basicConfig(level=logging.INFO)


    def generate_path(self, start_pose, goal_coordinate, update_callback):

        (start_x, start_y), theta = start_pose
        goal_x, goal_y = goal_coordinate

        reached_goal = False

        while not reached_goal:
            # Generate a random point
            new_point_x, new_point_y = self.generate_random_point(goal_coordinate)

            if self.obstacle_map.is_coordinate_occupied((new_point_x, new_point_y)):
                continue

            # Figure out which node to connect it to
            # by calculating the overall cost of the resulting path segment
                # IE cost of curve that would connect it to the existing curve + cost of already-established curve
            # Also, do not connect it to a node if the resulting curve would intersect an obstacle

            # Then, call our callback
            new_curve_points = []
            update_callback(new_curve_points)

            reached_goal = self._get_distance((new_point_x, new_point_y), goal_coordinate) < GOAL_THRESHOLD


    def generate_random_point(self, goal_coordinate):
        randomly_chose_goal = random.random() < 0.2

        if randomly_chose_goal:
            return goal_coordinate

        else:
            new_point_x = 3.0 * random.random()
            new_point_y = 2.0 * random.random()
            return new_point_x, new_point_y


    def connect_with_curve(self, p1, theta1, p2):
        x1, y1 = p1
        x2, y2 = p2
        slope_towards_center = math.tan(theta1 + math.pi/2)
        intercept = y1 - slope_towards_center * x1

        a1 = 1 / intercept
        b1 = -slope_towards_center / intercept

        slope_bw_points = y2-y1 / x2-x1
        slope_eq_points = -1 * slope_bw_points**-1
        x_mid = (x2 + x1) / 2
        y_mid = (y2 + y1) / 2
        intercept_eq = y_mid - slope_eq_points * x_mid

        a2 = 1 / intercept_eq
        b2 = -slope_eq_points / intercept_eq

        x_intersect = (b1 - b2) / (a1 - a2)
        y_intersect = (a2 - a1) / (b2 - b1)

        radius = self._get_distance(p1, (x_intersect, y_intersect))
        if radius < MIN_CURVATURE_RADIUS:
            return []

        angle_to_p1 = math.atan2(x1-x_intersect, y1-y_intersect)
        angle_to_p2 = math.atan2(x2-x_intersect, y2-y_intersect)

        points = []
        for i in range(20):
            angle = min(angle_to_p1, angle_to_p2) + abs(angle_to_p1 - angle_to_p2) / 20
            new_x = x_intersect + radius * math.cos(angle)
            new_y = y_intersect + radius * math.sin(angle)

            points.append((new_x, new_y))

        return points


    def _get_distance(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx**2 + dy**2)
