import math
import numpy
import time

from node_description import NodeDescription

import cv2

WAITKEY = 1


def _get_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def draw_line(a, b):
    # ax + by = 1
    if abs(b) < 0.00001:
        cv2.line(canvas, (int(1/a), 0), (int(1/a), 100), (255, 0, 0), 2)
        cv2.imshow("Canvas", canvas)
        cv2.waitKey(WAITKEY)
        return
    else:
        intercept = 1 / b

    if abs(a) < 0.0001:
        cv2.line(canvas, (0, int(1/b)), (100, int(1/b)), (255, 0, 0), 2)
        cv2.imshow("Canvas", canvas)
        cv2.waitKey(WAITKEY)
        return
    else:
        x_intercept = 1 / a

    if a == b:
        cv2.line(canvas, (0, 0), (100, 100), (255, 0, 0), 2)
        cv2.imshow("Canvas", canvas)
        cv2.waitKey(WAITKEY)
        return

    # p1 = ((1 - b*intercept)/a, intercept)
    # p2 = (x_intercept, (1 - a*x_intercept)/b)

    intercept = min(intercept, 100)
    x_intercept = min(x_intercept, 100)

    p1 = (0, intercept)
    p2 = (x_intercept, 0)

    p1 = (int(p1[0]), int(p1[1]))
    p2 = (int(p2[0]), int(p2[1]))

    cv2.line(canvas, p1, p2, (255, 0, 0), 2)
    cv2.imshow("Canvas", canvas)
    cv2.waitKey(WAITKEY)


def draw_circle(x, y):
    x = int(x)
    y = int(y)
    cv2.circle(canvas, (x, y), 2, (0, 0, 255), -1)
    cv2.imshow("Canvas", canvas)
    cv2.waitKey(WAITKEY)


def _reverse_points_if_necessary(points, start_point):
    first_pt = points[0]
    last_pt = points[-1]
    first_dist_to_pt = _get_distance(first_pt.coordinates, start_point)
    last_dist_to_pt = _get_distance(last_pt.coordinates, start_point)

    if first_dist_to_pt > last_dist_to_pt:
        points.reverse()


def connect_with_curve(p1, theta1, p2):

    draw_circle(*p1)
    draw_circle(*p2)
    draw_circle(50, 50)

    x1, y1 = p1
    x2, y2 = p2
    slope_towards_center = math.tan(theta1 * math.pi/180 + math.pi/2)
    intercept = y1 - slope_towards_center * x1

    a1 = -slope_towards_center / intercept
    b1 = 1 / intercept

    draw_line(a1, b1)

    x_mid = (x2 + x1) / 2
    y_mid = (y2 + y1) / 2
    slope_bw_points = (y2-y1) / (x2-x1)
    if slope_bw_points == 0:
        a2 = 1 / x_mid
        b2 = 0
    else:
        slope_eq_points = -1 * slope_bw_points**-1

        intercept_eq = y_mid - slope_eq_points * x_mid

        a2 = -slope_eq_points / intercept_eq
        b2 = 1 / intercept_eq

    draw_line(a2, b2)

    x_intersect = (b2 - b1) / (a1*b2 - a2*b1)
    y_intersect = (a1 - a2) / (a1*b2 - a2*b1)

    radius = _get_distance(p1, (x_intersect, y_intersect))
    if radius < 0.5:
        return [], -1

    angle_to_p1 = math.atan2(y1-y_intersect, x1-x_intersect)
    angle_to_p2 = math.atan2(y2-y_intersect, x2-x_intersect)

    points = []
    print("Angle to p1: " + str(angle_to_p1))
    for i in range(20):

        first_angle = angle_to_p1
        d_angle = (angle_to_p2 - angle_to_p1) / 20
        angle = first_angle + i * d_angle

        # angle = min(angle_to_p1, angle_to_p2) + i * abs(angle_to_p1 - angle_to_p2) / 20
        new_x = x_intersect + radius * math.cos(angle)
        new_y = y_intersect + radius * math.sin(angle)

        tip_angle = angle + (math.pi / 2) * (d_angle/abs(d_angle))
        tip_degrees = tip_angle * 180 / math.pi
        print("Tip angle: " + str(tip_degrees))

        new_node = NodeDescription((new_x, new_y), tip_degrees)

        points.append(new_node)

    _reverse_points_if_necessary(points, p1)

    for pt in points:
        draw_circle(*pt.coordinates)

    return points, radius * abs(angle_to_p1 - angle_to_p2)


canvas = numpy.zeros((100, 100, 3), dtype=numpy.uint8)


points, arc_length = connect_with_curve((25, 50), 0.01, (75, 60))

final_angle = points[-1].theta
print("Final angle: " + str(final_angle))
print("Waiting now.")
time.sleep(3)
