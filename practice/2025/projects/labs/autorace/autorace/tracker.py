import math

import cv2
import numpy as np

from rclpy.impl.rcutils_logger import RcutilsLogger


class ObjectTracker:

    WINDOW_NAME = 'Tracked Red Balls'

    def __init__(self, logger: RcutilsLogger):
        self.logger = logger
        self.found = 0

    def track(self, cv_image):
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 255])

        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160, 100, 20])
        upper2 = np.array([179, 255, 255])

        lower_mask = cv2.inRange(hsv, lower1, upper1)
        upper_mask = cv2.inRange(hsv, lower2, upper2)

        mask = lower_mask + upper_mask

        # some morphological operations (closing) to remove small blobs
        erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
        eroded_mask = cv2.erode(mask, erode_element)
        dilated_mask = cv2.dilate(eroded_mask, dilate_element)

        # cv2.imshow("Mask", dilated_mask)

        found_circles = cv2.HoughCircles(dilated_mask, cv2.HOUGH_GRADIENT, 1.0, 30,
                                            param1=100, param2=15, minRadius=2, maxRadius=200)

        if found_circles is not None:
            found_circles = np.uint16(np.around(found_circles))
            sorted_circles = sorted(found_circles[0, :], key=lambda circle: -circle[2])

            filtered_circles = []
            for circle_candidate in sorted_circles:
                if not filtered_circles:
                    filtered_circles.append(circle_candidate)
                    continue

                (xc, yc, rc, *_) = circle_candidate
                approved = False
                for (x, y, r, *_) in filtered_circles:
                    dist = abs(math.dist((xc, yc), (x, y)))
                    approved = dist > r+rc

                if approved:
                    filtered_circles.append(circle_candidate)

            found = len(filtered_circles)
            if self.found != found:
                self.found = found
                self.logger.info(f'Found {self.found} balls')

            circled_image = cv_image
            for circle in filtered_circles:
                circled_image = cv2.circle(cv_image, (circle[0], circle[1]), circle[2], (0, 255, 0), thickness=3)
            cv2.imshow(self.WINDOW_NAME, circled_image)
        else:
            if self.found != 0:
                self.found = 0
                self.logger.info(f'No balls found')

            cv2.imshow(self.WINDOW_NAME, cv_image)

        cv2.waitKey(3)
