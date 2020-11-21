import numpy as np
import cv2


def detect(image, low, high):
    mask = cv2.inRange(image, low, high)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
        print("can't find the target {},{}".format(low, high))
        return np.array([-1, -1])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])


def detect_red(image):
    return detect(image, (0, 0, 100), (5, 5, 255))


def detect_blue(image):
    return detect(image, (100, 0, 0), (255, 5, 5))


def detect_green(image):
    return detect(image, (0, 100, 0), (5, 255, 5))


def detect_yellow(image):
    return detect(image, (0, 100, 100), (5, 150, 150))


def detect_orange(image):
    mask = cv2.inRange(image, (0, 30, 90), (20, 85, 140))
    return mask


def find_target(image, template):
    res = cv2.matchTemplate(image, template, 0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    return np.array([min_loc[0], min_loc[1]])


def pixel2meter(image):
    circle1Pos = detect_blue(image)
    circle2Pos = detect_yellow(image)
    dist = np.sum((circle1Pos - circle2Pos) ** 2)
    # TODO Confirm the l is 3
    return 2.5 / np.sqrt(dist)
