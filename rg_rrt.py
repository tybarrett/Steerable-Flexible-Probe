import logging
import random
import math

import closed_nodes_maintainer
from node_description import NodeDescription


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

        start_node = NodeDescription((start_x, start_y), theta)
        existing_nodes = [start_node]

        last_node = None

        while not reached_goal:
            # Generate a random point
            new_point = self.generate_random_point(goal_coordinate)
            new_point_x, new_point_y = new_point

            print("New Point: " + str(new_point))

            # TODO - trim existing_nodes first before sorting it
            sorted_existing = sorted(existing_nodes, key=lambda node: self._get_distance(node.coordinates, new_point))

            points_and_lengths = []
            for existing_node in sorted_existing:

                connecting_points, arc_length = self.connect_with_curve(existing_node, new_point)

                if connecting_points:

                    goes_through_obstacle = False
                    for point in connecting_points:
                        if self.obstacle_map.is_coordinate_occupied(self.obstacle_map.img, point.coordinates[0], point.coordinates[1]):
                            goes_through_obstacle = True
                            print("Warning - this arc goes through an obstacle")
                            break

                    if not goes_through_obstacle:
                        points_and_lengths.append((connecting_points, arc_length))

            # sorted(points_and_lengths, key=lambda x: x[1])
            if len(points_and_lengths):
                print("Found a path that is " + str(points_and_lengths[0][1]) + " units long")
                shortest_path = points_and_lengths[0][0]
                terminal_node = shortest_path[-1]
                existing_nodes.append(terminal_node)
                print("Adding a new node to the existing_nodes with a theta of " + str(terminal_node.theta))
                update_callback(points_and_lengths[0][0])

                if new_point == goal_coordinate:
                    return self.backtrack(terminal_node), 0 # TODO - put a cost here

        return solved_trees


    def generate_random_point(self, goal_coordinate):
        randomly_chose_goal = random.random() < 0.2

        if randomly_chose_goal:
            return goal_coordinate

        else:
            new_point_x = 206 * random.random()
            new_point_y = 1.0 * random.random()
            return new_point_x, new_point_y

    def connect_with_curve(self, p1, p2):

        x1, y1 = p1.coordinates
        theta1 = p1.theta
        x2, y2 = p2
        slope_towards_center = math.tan(theta1 * math.pi / 180 + math.pi / 2)
        intercept = y1 - slope_towards_center * x1

        a1 = -slope_towards_center / intercept
        b1 = 1 / intercept

        x_mid = (x2 + x1) / 2
        y_mid = (y2 + y1) / 2
        slope_bw_points = (y2 - y1) / (x2 - x1)
        if slope_bw_points == 0:
            a2 = 1 / x_mid
            b2 = 0
        else:
            slope_eq_points = -1 * slope_bw_points ** -1

            intercept_eq = y_mid - slope_eq_points * x_mid

            a2 = -slope_eq_points / intercept_eq
            b2 = 1 / intercept_eq

        x_intersect = (b2 - b1) / (a1 * b2 - a2 * b1)
        y_intersect = (a1 - a2) / (a1 * b2 - a2 * b1)

        radius = self._get_distance(p1.coordinates, (x_intersect, y_intersect))
        if radius < 0.5:
            return [], -1

        angle_to_p1 = math.atan2(y1 - y_intersect, x1 - x_intersect)
        angle_to_p2 = math.atan2(y2 - y_intersect, x2 - x_intersect)

        points = []
        first_angle = angle_to_p1
        d_angle = (angle_to_p2 - angle_to_p1) / 40

        # Check if we are moving in the opposite direction of the tip
        tip_angle = first_angle + (math.pi / 2) * (d_angle / abs(d_angle))
        if abs(tip_angle - theta1 * math.pi / 180) > math.pi / 2:
            return [], -1

        for i in range(40):

            angle = first_angle + i * d_angle

            new_x = x_intersect + radius * math.cos(angle)
            new_y = y_intersect + radius * math.sin(angle)

            tip_angle = angle + (math.pi / 2) * (d_angle/abs(d_angle))
            tip_degrees = tip_angle * 180 / (math.pi * 2)

            new_node = NodeDescription((new_x, new_y), tip_degrees)

            points.append(new_node)

        self._reverse_points_if_necessary(points, p1.coordinates)

        prev_node = p1
        for pt in points:
            pt.parent_node = prev_node
            prev_node = pt

        return points, radius * abs(angle_to_p1 - angle_to_p2)


    def _reverse_points_if_necessary(self, points, start_point):
        first_pt = points[0]
        last_pt = points[-1]
        first_dist_to_pt = self._get_distance(first_pt.coordinates, start_point)
        last_dist_to_pt = self._get_distance(last_pt.coordinates, start_point)

        if first_dist_to_pt > last_dist_to_pt:
            points.reverse()


    def _get_distance(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx**2 + dy**2)


    def backtrack(self, goal_node):
        backtrack_nodes = []
        this_node = goal_node
        while this_node:
            backtrack_nodes.insert(0, this_node)
            this_node = this_node.parent_node
        return backtrack_nodes