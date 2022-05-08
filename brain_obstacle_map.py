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
import cv2
import numpy as np


IN_MARGIN = 1
IN_OBSTACLE = 2
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
# Obstacle map colors
ACCESSIBLE = (0, 0, 0)
COMMON = (50, 50, 50)
CAREFUL = (51, 51, 51)
WARNING = (128, 128, 128)
DANGEROUS = (179, 179, 179)
AVOID = WHITE
# scaleX is scale factor in x direction
# scaleY is scale factor in y direction
SCALE_X = 0.8
SCALE_Y = 0.8
START = (50, 50)
GOAL = (200, 200)
COST = None
test_coord_x = 200
test_coord_y = 240
# This class is specific to Project 5


class BrainObstacleMap(object):
    def __init__(self, img, start, goal):
        self.img = img
        self.height, self.width, _ = self.img.shape
        self.start = start
        self.goal = goal

        self.contours_drawn = None

    def detect_boundary_for_brain(self, image, bin_img):
        imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 0, 255, cv2.THRESH_BINARY)
        # cv2.imshow('thresh', thresh)
        # cv2.waitKey(0)
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(bin_img, contours[1:5], -1, BLUE, 2)

        white_img = np.zeros([828, 828, 3], dtype=np.uint8)
        white_img.fill(255)
        stencil = np.zeros(white_img.shape).astype(white_img.dtype)
        color = BLUE
        cv2.fillPoly(stencil, contours[0:4], color)
        self.inside_map = cv2.bitwise_and(white_img, stencil)
        # cv2.imshow("result_final", result)

        return contours  # returns contours drawn

    def scale_img(self):
        # scales up the normal image input
        scaleUpImg_input = cv2.resize(self.img, None, fx=SCALE_X*5,
                                      fy=SCALE_Y*5, interpolation=cv2.INTER_LINEAR)

        # cv2.imshow("Scaled Up Input Image", scaleUpImg_input)

        ret, bin_img = cv2.threshold(self.img, 100, 255, cv2.THRESH_BINARY)
        scaleUpImg = cv2.resize(bin_img, None, fx=SCALE_X*5,
                                fy=SCALE_Y*5, interpolation=cv2.INTER_LINEAR)

        print('Scaled image x: ', scaleUpImg.shape[0])
        print('Scaled image y: ', scaleUpImg.shape[1])
        # cv2.imshow("Scaled Up Binary Image", scaleUpImg)
        return scaleUpImg_input, scaleUpImg

    def is_coordinate_occupied(self, scaled_img, x, y):

        # if x < self.clearance_margin or x > self.width - self.clearance_margin:
        # #     return IN_MARGIN
        # if y < self.clearance_margin or y > self.height - self.clearance_margin:
        #     return IN_MARGIN
        # + TO-DO Add margin and goal point
        x = int(x)
        y = int(y)

        if x < 0 or x > scaled_img.shape[1] - 1:
            return True
        if y < 0 or y > scaled_img.shape[0] - 1:
            return True

        if tuple(scaled_img[y, x]) == RED:
            # print('Point in Tumor...!')
            # print('x and y coordinates for Tumor are: ', (x, y))
            return True
        if tuple(scaled_img[y, x]) == GREEN:
            # print('This is the Start Point...!')
            # print('x and y coordinates for point are: ', (x, y))
            return True
        if tuple(scaled_img[y, x]) == WHITE:
            # print('Point in Obstacle...!')
            # print('x and y coordinates for Obstacle are: ', (x, y))
            return True
        else:
            # print('point is in Free Space...!')
            # print('x and y coordinates for free point: ', (x, y))
            return False

    def get_cost_for_coordinate(self, scaled_img, x, y):
        '''
        returns cost for pixel between 0-1, 1 being a 
        'no-go' area and 0 being 'good-to-go' area

        Accessible - 0
        Common - 0.1
        Careful - 0.7
        Warning - 0.8
        Dangerous - 0.9
        Avoid - 1
        Other regions - 0.6

        '''

        x = int(x)
        y = int(y)

        if tuple(scaled_img[y, x]) == ACCESSIBLE:
            COST = 0
            # print('Point in Accessible Space...!')
        elif tuple(scaled_img[y, x]) == COMMON:
            COST = 0.1
            # print('Point is in Common Space...!')
        elif tuple(scaled_img[y, x]) == CAREFUL:
            COST = 0.7
            # print('Point in Careful space...!')
        elif tuple(scaled_img[y, x]) == WARNING:
            COST = 0.8
            # print('Point is in Warning Space...!')
        elif tuple(scaled_img[y, x]) == DANGEROUS:
            COST = 0.9
            # print('Point is in Dangerous Space...!')
        elif tuple(scaled_img[y, x]) == AVOID:
            COST = 1
            # print('Point is in Avoid Space...!')
        elif tuple(scaled_img[y, x]) == RED:
            COST = 0
            # print('Point is in the Tumor...!')
        elif tuple(scaled_img[y, x]) == GREEN:
            COST = 0
            # print('Point is in Start Position...!')
        else:
            COST = 0.6
            # print('point is in other region...!')

        # print('x and y coordinates for point are: ', (x, y))
        return COST

    def is_coordinate_inside_brain(self, contours, x, y):
        # Make new brain image with filled contours
        # white image

        # white_img = np.zeros([828, 828, 3], dtype=np.uint8)
        # white_img.fill(255)
        # stencil = np.zeros(white_img.shape).astype(white_img.dtype)
        # color = BLUE
        # cv2.fillPoly(stencil, contours[0:4], color)
        # result = cv2.bitwise_and(white_img, stencil)

        # cv2.imshow("result_final", result)
        # cv2.waitKey(0)

        # check if coordinate is inside the brain or outside
        if x < 0 or x > self.inside_map.shape[1] - 1:
            return False
        if y < 0 or y > self.inside_map.shape[0] - 1:
            return False

        if tuple(self.inside_map[y, x]) == BLUE:
            # print('Point inside Brain...!')
            return True
        elif tuple(self.inside_map[y, x]) == BLACK:
            # print('Point is outside the Brain...!')
            return False
        else:
            # print('Point is not able to identify its location...!')
            return None

    def is_coordinate_inside(self, test_coord_x, test_coord_y):

        test_coord_x = int(test_coord_x)
        test_coord_y = int(test_coord_y)

        if self.contours_drawn == None:
            self.scaled_img = self.scale_img()
            self.contours_drawn = self.detect_boundary_for_brain(self.scaled_img[0], self.scaled_img[1])
        return self.is_coordinate_inside_brain(self.contours_drawn, test_coord_x*4, test_coord_y*4)



if __name__ == "__main__":
    x_s, y_s = START
    x_g, y_g = GOAL
    input_img = cv2.imread('data/brain.png')
    map = BrainObstacleMap(input_img, START, GOAL)
    scaled_img = map.scale_img()
    contours_drawn = map.detect_boundary_for_brain(
        scaled_img[0], scaled_img[1])
    print('LOCATION :')    
    map.is_coordinate_inside_brain(contours_drawn, test_coord_x, test_coord_y)
    scaled_img[0][x_s, y_s] = GREEN  # START SET input img
    scaled_img[0][x_g, y_g] = RED  # GOAL SET
    scaled_img[1][x_s, y_s] = GREEN  # START SET binary img
    scaled_img[1][x_g, y_g] = RED  # GOAL SET
    cv2.imshow("Scaled Up Input Image", scaled_img[1])
   
    print('STATUS :')
    result = map.is_coordinate_occupied(
        scaled_img[1], test_coord_x, test_coord_y)
    print("Is coordinate occupied?")
    print(result)
    cost = map.get_cost_for_coordinate(
        scaled_img[0], test_coord_x, test_coord_y)
    
    print('VALUE :')    
    print("Cost of the coordinate?")
    print(cost)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
