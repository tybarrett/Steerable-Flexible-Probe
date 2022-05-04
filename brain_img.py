import numpy as np
import cv2
from matplotlib import pyplot as plt

# print(cv2.__version__)
# scaleX is scale factor in x direction
# scaleY is scale factor in y direction
scaleX = 0.8
scaleY = 0.8
RED = (0, 0, 255)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)


input_img = cv2.imread('Steerable-Flexible-Probe/data/brain.png')
cv2.imshow("original image", input_img)
ret, img = cv2.threshold(input_img, 100, 255, cv2.THRESH_BINARY)
img[50, 50] = RED
scaleUpImg = cv2.resize(img, None, fx=scaleX*5,
                        fy=scaleY*5, interpolation=cv2.INTER_LINEAR)

cv2.imshow("Scaled Up", scaleUpImg)
# scaleUpImg[50,50] = RED
print('Scaled image x: ', scaleUpImg.shape[0])
print('Scaled image y: ', scaleUpImg.shape[1])


def is_point_in_obstacle(x, y):
    if tuple(img[x, y]) == RED:
        print('point in Tumor!!!!')
        print('x and y coordinates for Tumor are: ', (x, y))
    if tuple(img[x, y]) == WHITE:
        print('point in Obstacle!!!!')
        print('x and y coordinates for Obstacle are: ', (x, y))
    else:
        print('point in free!!!!')
        print('x and y coordinates for free point: ', (x, y))     


is_point_in_obstacle(120, 120)

cv2.waitKey(0)
cv2.destroyAllWindows()

if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()
