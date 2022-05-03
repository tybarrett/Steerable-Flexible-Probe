"""node_description.py - A data model that contains information for a given node in a path traversal algorithm."""


import math


DISTANCE_THRESHOLD = 0.5
THETA_THRESHOLD = 30

class NodeDescription(object):
    def __init__(self, coordinates, theta, cost_to_come=0, cost_to_go=0, parent_node=None, ul=0, ur=0):
        self.coordinates = coordinates
        self.theta = theta
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.parent_node = parent_node

        self.ul = ul
        self.ur = ur


    def get_cost(self):
        return self.cost_to_come + self.cost_to_go


    def __eq__(self, other):
        dx = self.coordinates[0] - other.coordinates[0]
        dy = self.coordinates[1] - other.coordinates[1]
        dtheta = abs(self.theta - other.theta)

        return dx == 0 and dy == 0 and dtheta == 0


    def __hash__(self):
        return hash(self.coordinates) + hash(self.theta)


    def __lt__(self, other):
        return self.coordinates < other.coordinates


    def __str__(self):
        return str(self.coordinates) + " - Theta: " + str(self.theta) + " - Cost: " + str(self.get_cost())
