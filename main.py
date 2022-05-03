"""main.py - The top-level code for ENPM661 Project 3"""


import time

import a_star
import obstacle_map
import visualization


# TODO - find this constant somewhere in the documentation
TURTLEBOT_ROBOT_RADIUS = 0.089


print("Please enter the start and goal poses as comma-separated integers, such as: 50, 220, 30")
start_point_str = input("Please input the starting point and orientation: ")
start_coord = start_point_str.split(",")
start_x = float(start_coord[0].strip())
start_y = float(start_coord[1].strip())
start_theta = float(start_coord[2].strip())
start_point = (start_x, start_y), start_theta

# start_point = (150, 100), 0

goal_point_str = input("Please input the goal point: ")
goal_coord = goal_point_str.split(",")
goal_x = float(goal_coord[0].strip())
goal_y = float(goal_coord[1].strip())
goal_point = (goal_x, goal_y)

# goal_point = (250, 100)

rpm_str = input("Please enter the desired rpms: ")
rpms = rpm_str.split(",")
rpm1 = int(rpms[0])
rpm2 = int(rpms[1])

clearance_str = input("Finally, please enter the desired clearance: ")
clearance = float(clearance_str.strip())

print("Start point: " + str(start_point))
print("Goal point: " + str(goal_point))


o_map = obstacle_map.generate_obstacle_map(clearance + TURTLEBOT_ROBOT_RADIUS)
# o_map = obstacle_map.generate_obstacle_map(0) # TODO - what should we put here?

visualizer = visualization.PathPlanningVisualizer()
visualizer.draw_obstacle_map(o_map)

a = a_star.AStar(o_map, goal_point, rpm1, rpm2)
t0 = time.time()
path, cost, motor_inputs = a.generate_path(start_point, goal_point, lambda x: visualizer.draw_visited_node(x))
if path:
    print("Found a path from start to goal point!")

print("Time required: " + str(time.time() - t0))

visualizer.draw_path(path)
visualizer.write_video_file()

# Allow enough time to finish flushing the buffer to the AVI output file.
time.sleep(2)

for motor_input in motor_inputs:
    print(motor_input)
