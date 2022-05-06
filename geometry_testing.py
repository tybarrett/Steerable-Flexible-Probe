import math
import numpy
import time
import random

from node_description import NodeDescription

import cv2


def _get_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def draw_line(a, b):
    # ax + by = 1
    intercept = 1 / b
    x_intercept = 1 / a

    # p1 = ((1 - b*intercept)/a, intercept)
    # p2 = (x_intercept, (1 - a*x_intercept)/b)

    p1 = (0, intercept)
    p2 = (x_intercept, 0)

    p1 = (int(p1[0]), int(p1[1]))
    p2 = (int(p2[0]), int(p2[1]))

    cv2.line(canvas, p1, p2, (255, 0, 0), 2)
    cv2.imshow("Canvas", canvas)
    cv2.waitKey(1)


def draw_circle(x, y):
    x = int(x)
    y = int(y)
    cv2.circle(canvas, (x, y), 2, (0, 0, 255), -1)
    cv2.imshow("Canvas", canvas)
    cv2.waitKey(1)


def connect_with_curve(p1, theta1, p2):

    draw_circle(*p1)
    draw_circle(*p2)
    draw_circle(50, 50)

    x1, y1 = p1
    x2, y2 = p2
    slope_towards_center = math.tan(theta1 * math.pi/180)
    intercept = y1 - slope_towards_center * x1

    a1 = -slope_towards_center / intercept
    b1 = 1 / intercept

    draw_line(a1, b1)

    slope_bw_points = (y2-y1) / (x2-x1)
    slope_eq_points = -1 * slope_bw_points**-1
    x_mid = (x2 + x1) / 2
    y_mid = (y2 + y1) / 2
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
    for i in range(20):
        angle = min(angle_to_p1, angle_to_p2) + i * abs(angle_to_p1 - angle_to_p2) / 20
        new_x = x_intersect + radius * math.cos(angle)
        new_y = y_intersect + radius * math.sin(angle)

        tip_angle = angle + math.pi / 2
        tip_degrees = tip_angle * 180 / (math.pi *2)

        new_node = NodeDescription((new_x, new_y), tip_degrees)
        draw_circle(*new_node.coordinates)

        points.append(new_node)

    return points, radius * abs(angle_to_p1 - angle_to_p2)


canvas = numpy.zeros((100, 100, 3), dtype=numpy.uint8)


connect_with_curve((10, 10), -45, (30, 90))
time.sleep(5)
