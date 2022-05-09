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
NUM_TREES = 10


TEST_CASES = [((10, 100), 0.01, (47, 50)),
              ((10, 100), 0.01, (153, 50)),
              ((29, 44), 45.01, (153, 50)),
              ((103, 30), 89.01, (153, 50)),
              # ((103, 30), 89.01, (60, 140)),
              ((186, 105), 180.01, (60, 115)),
              ((15, 114), -25, (78, 72))]

test_case = TEST_CASES[-1]
start_point = test_case[:2]
goal_point = test_case[2]

print("Start point: " + str(start_point))
print("Goal point: " + str(goal_point))

# o_map = obstacle_map.generate_obstacle_map(PROBE_RADIUS_M)
# o_map = obstacle_map.generate_obstacle_map(0.1)
input_img = cv2.imread("data/brain.png")
o_map = brain_obstacle_map.BrainObstacleMap(input_img, None, None)

visualizer = visualization.PathPlanningVisualizer()
visualizer.draw_obstacle_map(o_map)


def execute_test_case(start_point, goal_point, reverse=False):
    a = rg_rrt.Rg_Rrt(o_map, LENGTH_WEIGHT, CLEARANCE_WEIGHT, RISK_WEIGHT)

    answers = []
    for solution_i in range(NUM_TREES):
        t0 = time.time()
        if reverse:
            solved_trees = a.original_research_generate_path((goal_point, 0), start_point[0], lambda x: None)
        else:
            solved_trees = a.generate_path(start_point, goal_point, lambda x: None)

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

    return sorted_trees


regular_trees = []
reverse_trees = []
for i, test_case in enumerate(TEST_CASES):
    print("Executing Test Case " + str(i) + " / " + str(len(TEST_CASES)))
    start_point = test_case[:2]
    goal_point = test_case[2]

    trees = execute_test_case(start_point, goal_point)
    regular_trees.extend(trees)

    trees = execute_test_case(start_point, goal_point, reverse=True)
    reverse_trees.extend(trees)


regular_average = 0
for tree in regular_trees:
    regular_average += tree[-1].get_cost()
regular_average /= len(regular_trees)

reverse_average = 0
for tree in reverse_trees:
    reverse_average += tree[-1].get_cost()
reverse_average /= len(reverse_trees)

print("Regular Average: " + str(regular_average))
print("Reverse Average: " + str(reverse_average))


# Allow enough time to finish flushing the buffer to the AVI output file.
time.sleep(2)
