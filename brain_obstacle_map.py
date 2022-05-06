"""brain_obstacle_map.py - Describes a map of the obstacles in the brain scan


image points = [y,x] horizontal --> x vertical --> y
Test points:
Goal/Tumour : 50,50
scaled up image
Goal/Tumour : 200,236-299 (240 is good), 236 is not white but a shade of it
Obstacle : 


# + TO-DO Set clearance for tumor?? pixel is center or boundary of tumor
"""


# import math
# import numpy as np
from tracemalloc import start
import cv2


IN_MARGIN = 1
IN_OBSTACLE = 2
RED = (0, 0, 255)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
# scaleX is scale factor in x direction
# scaleY is scale factor in y direction
SCALE_X = 0.8
SCALE_Y = 0.8
START = (50, 50)
GOAL = (200, 200)

# This class is specific to Project 5


class BrainObstacleMap(object):
    def __init__(self, img, start, goal):
        self.img = img
        self.start = start
        self.goal = goal

    def scale_img(self):
        x_s, y_s = self.start
        x_g, y_g = self.goal

        ret, bin_img = cv2.threshold(self.img, 100, 255, cv2.THRESH_BINARY)
        scaleUpImg = cv2.resize(bin_img, None, fx=SCALE_X*5,
                                fy=SCALE_Y*5, interpolation=cv2.INTER_LINEAR)

        print('Scaled image x: ', scaleUpImg.shape[0])
        print('Scaled image y: ', scaleUpImg.shape[1])
        scaleUpImg[x_s, y_s] = GREEN  # START SET
        scaleUpImg[x_g, y_g] = RED  # GOAL SET
        cv2.imshow("Scaled Up", scaleUpImg)
        return scaleUpImg

    def is_coordinate_occupied(self, scaled_img, x, y):

        # if x < self.clearance_margin or x > self.width - self.clearance_margin:
        # #     return IN_MARGIN
        # if y < self.clearance_margin or y > self.height - self.clearance_margin:
        #     return IN_MARGIN
        # + TO-DO Add margin and goal point
        if tuple(scaled_img[x, y]) == RED:
            # TO-DO Add range of reds to fetect tumour? Scale up causes color to expand aroudn pixels
            print('Point in Tumor...!')
            print('x and y coordinates for Tumor are: ', (x, y))
            return True
        if tuple(scaled_img[x, y]) == GREEN:
            # TO-DO Add range of reds to fetect tumour? Scale up causes color to expand aroudn pixels
            print('This is the Start Point...!')
            print('x and y coordinates for Tumor are: ', (x, y))
            return True    
        if tuple(scaled_img[x, y]) == WHITE:
            print('Point in Obstacle...!')
            print('x and y coordinates for Obstacle are: ', (x, y))
            return True
        else:
            print('point is in Free Space...!')
            print('x and y coordinates for free point: ', (x, y))
            return False


if __name__ == "__main__":
    input_img = cv2.imread('Steerable-Flexible-Probe/data/brain.png')
    cv2.imshow("original image", input_img)
    map = BrainObstacleMap(input_img, START, GOAL)
    scaled_img = map.scale_img()
    result = map.is_coordinate_occupied(scaled_img, 50, 50)
    print("Is coordinate occupied?")
    print(result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
