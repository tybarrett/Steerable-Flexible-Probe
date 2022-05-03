import logging

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
        return # TODO
