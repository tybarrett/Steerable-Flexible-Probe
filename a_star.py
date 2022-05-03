"""A_star.py - Implementation of A_Star for Project 3."""


import time
import math
import heapq
import logging

import diff_drive_traverser
import node_description
import closed_nodes_maintainer


GOAL_THRESHOLD = 0.15


class AStar(object):
    def __init__(self, obstacle_map, goal_coordinate, rpm1, rpm2):
        self.obstacle_map = obstacle_map
        self.open = []
        self.closed = closed_nodes_maintainer.ClosedNodesMaintainer(0.5, self.obstacle_map.width, self.obstacle_map.height)

        self.traverser = diff_drive_traverser.DiffDriveTraverser(obstacle_map, goal_coordinate, rpm1, rpm2)

        self.coord_to_lowest_cost = {}

        logging.basicConfig(level=logging.INFO)


    def _calculate_distance(self, coord1, coord2):
        dx = coord1[0] - coord2[0]
        dy = coord1[1] - coord2[1]
        return math.sqrt(dx**2 + dy**2)


    def generate_path(self, start_pose, goal_pose, update_callback):

        start_coordinate, start_theta = start_pose
        goal_coordinate = goal_pose

        if self.obstacle_map.is_coordinate_occupied(start_coordinate):
            print("The starting coordinate " + str(start_coordinate) + " is inside an obstacle!")
            return [], -1, []

        if self.obstacle_map.is_coordinate_occupied(goal_coordinate):
            print("The goal coordinate " + str(goal_coordinate) + " is inside an obstacle!")
            return [], -1, []

        dist_to_goal = self._calculate_distance(start_coordinate, goal_coordinate)
        start_node = node_description.NodeDescription(start_coordinate,
                                                      theta=start_theta,
                                                      cost_to_come=0,
                                                      cost_to_go=dist_to_goal,
                                                      parent_node=None)

        heapq.heappush(self.open, (start_node.get_cost(), [start_node]))

        # Get neighbor nodes
        while True: # TODO - stop when the heap is empty

            _, these_nodes = heapq.heappop(self.open)
            this_node = these_nodes[-1]

            t = time.time()
            if self.closed.is_there_duplicate(this_node):
                continue
            logging.debug("Time required to check if this node is a duplicate: " + str(time.time() - t))

            t = time.time()
            self.closed.record_new_node(this_node)
            logging.debug("Time required to record this node as a duplicate: " + str(time.time() - t))

            t = time.time()
            for route_node in these_nodes:
                if route_node.coordinates not in self.coord_to_lowest_cost:
                    self.coord_to_lowest_cost[route_node.coordinates] = route_node
                elif self.coord_to_lowest_cost[route_node.coordinates].get_cost() > route_node.get_cost():
                    self.coord_to_lowest_cost[route_node.coordinates] = route_node
            logging.debug("Time required to put this node in the lowest cost dictionary: " + str(time.time() - t))

            t = time.time()
            dist_to_goal = self._calculate_distance(this_node.coordinates, goal_coordinate)
            if dist_to_goal < GOAL_THRESHOLD:
                return self.backtrack(this_node)
            logging.debug("Time required to check if it's the goal point: " + str(time.time() - t))

            t = time.time()
            free_path_options = self.traverser.get_valid_neighbors(this_node)
            print("Number of options: " + str(len(free_path_options)))
            logging.debug("Time required to get the neighbors: " + str(time.time() - t))

            t = time.time()
            for option in free_path_options:
                update_callback(option)
            logging.debug("Time required to run our vis callback: " + str(time.time() - t))

            # t0 = time.time()
            # t = time.time()
            # new_free_neighbors = [x for x in free_neighbors if not self.closed.is_there_duplicate(x)]
            # logging.debug("Time required to trim the duplicate neighbors: " + str(time.time() - t))
            # print("Time taken to check for duplicates: " + str(time.time() - t0))
            # new_free_neighbors = []
            # for neighbor in free_neighbors:
            #     is_in_closed = False
            #     for closed_node in self.closed:
            #         is_close = self._calculate_distance(neighbor.coordinates, closed_node.coordinates) <= 0.5
            #         rotation_close = abs(neighbor.theta - closed_node.theta) <= 30
            #         if is_close and rotation_close:
            #             is_in_closed = True
            #     if not is_in_closed:
            #         new_free_neighbors.append(neighbor)

            for option in free_path_options:
                next_node = option[-1]
                heapq.heappush(self.open, (next_node.get_cost(), option))


    def backtrack(self, goal_node):
        backtrack_nodes = []
        motor_inputs = []
        this_node = goal_node
        while this_node:
            backtrack_nodes.insert(0, this_node)
            motor_inputs.insert(0, (this_node.ul, this_node.ur))
            this_node = self.coord_to_lowest_cost[this_node.coordinates].parent_node

        return backtrack_nodes, goal_node.get_cost(), motor_inputs


if __name__ == '__main__':
    import obstacle_map
    o_map = obstacle_map.generate_obstacle_map(0)

    a = AStar(o_map, 5, (20, 20))
    path, cost = a.generate_path(((1, 1), 0), (20, 20), lambda x: None)

    print("Cost to get from start point to goal point: " + str(cost))
    print("The path:")
    for node in path:
        print(node)
