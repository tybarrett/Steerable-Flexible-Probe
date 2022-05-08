"""main.py - The top-level code for ENPM661 Project 5"""


import time

import a_star
import obstacle_map
import rg_rrt
import visualization


PROBE_RADIUS_M = 0.002

LENGTH_WEIGHT = 1.0
CLEARANCE_WEIGHT = 1.0
RISK_WEIGHT = 1.0


# print("Please enter the start and goal poses as comma-separated integers, such as: 50, 220, 30")
# start_point_str = input("Please input the starting point and orientation: ")
# start_coord = start_point_str.split(",")
# start_x = float(start_coord[0].strip())
# start_y = float(start_coord[1].strip())
# start_theta = float(start_coord[2].strip())
# start_point = (start_x, start_y), start_theta

start_point = (0, 0.5), 0.01

# goal_point_str = input("Please input the goal point: ")
# goal_coord = goal_point_str.split(",")
# goal_x = float(goal_coord[0].strip())
# goal_y = float(goal_coord[1].strip())
# goal_point = (goal_x, goal_y)

goal_point = (1, 0.8)

print("Start point: " + str(start_point))
print("Goal point: " + str(goal_point))

o_map = obstacle_map.generate_obstacle_map(PROBE_RADIUS_M)

visualizer = visualization.PathPlanningVisualizer()
visualizer.draw_obstacle_map(o_map)

a = rg_rrt.Rg_Rrt(o_map, LENGTH_WEIGHT, CLEARANCE_WEIGHT, RISK_WEIGHT)
t0 = time.time()
path, cost = a.generate_path(start_point, goal_point, lambda x: visualizer.draw_visited_node(x))
if path:
    print("Found a path from start to goal point!")

print("Time required: " + str(time.time() - t0))

visualizer.draw_path(path)
visualizer.write_video_file()

# Allow enough time to finish flushing the buffer to the AVI output file.
time.sleep(2)
