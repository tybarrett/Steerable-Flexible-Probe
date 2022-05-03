"""Map_Traverser.py - Contains methods for traversing the map in 8 directions."""


import visualization
import math
import node_description
import obstacle_map


WHEEL_RADIUS = 0.038
WHEEL_DISTANCE = 0.354


class DiffDriveTraverser(object):
    def __init__(self, obstacle_map, goal_coordinate, rpm1, rpm2):
        self.obstacle_map = obstacle_map
        self.goal_coord = goal_coordinate

        self.options = (0, rpm1, rpm2)
        self.potential_outputs = []
        for a in self.options:
            for b in self.options:
                self.potential_outputs.append((a, b))


    def get_valid_neighbors(self, node):
        valid_neighbors = []
        for output in self.potential_outputs:
            ul, ur = output

            points = self.generate_points(node, ul, ur)
            if points:
                valid_neighbors.append(points)

        return valid_neighbors

    def generate_points(self, node, ul, ur):
        x, y = node.coordinates
        theta = node.theta

        last_parent_node = node

        points = []
        for i in range(5): # TODO - arbitrarily chosen
            x += 0.5*WHEEL_RADIUS * (ul + ur) * math.cos(theta * 3.14 / 180)
            y += 0.5*WHEEL_RADIUS * (ul + ur) * math.sin(theta * 3.14 / 180)
            theta += ((WHEEL_RADIUS / WHEEL_DISTANCE) * (ur - ul)) * 180 / math.pi

            if not self.obstacle_map.is_coordinate_occupied((x, y)):
                new_cost_to_come = node.cost_to_come + (0.5*WHEEL_RADIUS * (ul + ur)) * i

                x_to_goal = self.goal_coord[0] - x
                y_to_goal = self.goal_coord[1] - y
                cost_to_go = math.sqrt(x_to_goal**2 + y_to_goal**2)

                new_node_description = node_description.NodeDescription((x, y),
                                                                        theta=theta,
                                                                        cost_to_come=new_cost_to_come,
                                                                        cost_to_go=cost_to_go,
                                                                        parent_node=last_parent_node,
                                                                        ul=ul,
                                                                        ur=ur)
                points.append(new_node_description)
                last_parent_node = new_node_description

            else:
                return None

        print(len(points))
        return points


if __name__ == "__main__":
    goal_coordinate = (100, 100)
    obstacle_map = obstacle_map.generate_obstacle_map(0)
    trav = DiffDriveTraverser(obstacle_map, goal_coordinate, 8, 10)

    start_x = 50
    start_y = 50
    theta = 0
    start_node = node_description.NodeDescription((start_x, start_y),
                                                                        theta=theta,
                                                                        cost_to_come=0,
                                                                        cost_to_go=0,
                                                                        parent_node=None)

    other_points = trav.get_valid_neighbors(start_node)

    viz = visualization.PathPlanningVisualizer()
    viz.draw_obstacle_map(obstacle_map)

    for point_seq in other_points:
        viz.draw_visited_node(point_seq)
    viz.write_video_file()