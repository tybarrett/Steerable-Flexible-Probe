"""main.py - The top-level code for ENPM661 Project 5"""


import time
import cv2

import brain_obstacle_map
import rg_rrt
import visualization


PROBE_RADIUS_M = 0.002

LENGTH_WEIGHT = 1.0
CLEARANCE_WEIGHT = 1.0
RISK_WEIGHT = 1.0

# NUM_TREES = 20
NUM_TREES = 1


TEST_CASES = [((10, 100), 0.01, (47, 50)),
              ((10, 100), 0.01, (153, 50)),
              ((29, 44), 45.01, (153, 50)),
              ((103, 30), 89.01, (153, 50)),
              ((103, 30), 89.01, (60, 150))]

start_point = (103, 30), 89.01

goal_point = (50, 153)

print("Start point: " + str(start_point))
print("Goal point: " + str(goal_point))

# o_map = obstacle_map.generate_obstacle_map(PROBE_RADIUS_M)
# o_map = obstacle_map.generate_obstacle_map(0.1)
input_img = cv2.imread("data/brain.png")
o_map = brain_obstacle_map.BrainObstacleMap(input_img, None, None)

visualizer = visualization.PathPlanningVisualizer()
visualizer.draw_obstacle_map(o_map)

a = rg_rrt.Rg_Rrt(o_map, LENGTH_WEIGHT, CLEARANCE_WEIGHT, RISK_WEIGHT)

answers = []
for solution_i in range(NUM_TREES):
    t0 = time.time()
    solved_trees = a.generate_path(start_point, goal_point, lambda x: visualizer.draw_visited_node(x))

    print("Found paths from start to goal point!")
    print("Time required: " + str(time.time() - t0))
    print("Found solution #" + str(solution_i + 1))
    for solved_tree in solved_trees:
        path = solved_tree
        if path:
            print("Total cost for the path: " + str(path[-1].get_cost()))
            visualizer.draw_visited_node(path)
            answers.append(solved_tree)

sorted_trees = sorted(answers, key=lambda x: x[-1].get_cost())
visualizer.draw_path(sorted_trees[0])

visualizer.write_video_file()

print("-"*20)
for tree in sorted_trees:
    print("Overall Cost: " + str(tree[-1].get_cost()))
    print("\tTree length: " + str(tree[-1].cost_to_come))
    print("\tTree risk: " + str(tree[-1].accumulated_risk))
    print()

# Allow enough time to finish flushing the buffer to the AVI output file.
time.sleep(2)
