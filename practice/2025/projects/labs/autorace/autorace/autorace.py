import numpy as np
import cv2
import math
import queue

from rclpy.impl.rcutils_logger import RcutilsLogger


class AutoRacer:

    def __init__(self, logger: RcutilsLogger):
        self.logger = logger
        self.queue_size = 8
        self.compensations = queue.Queue(maxsize=self.queue_size)

    def get_compensation(self, cv_image):
        border_yellow, border_white = self.detect_lines(cv_image)

        cv2.line(cv_image, (border_yellow, 0), (border_yellow, cv_image.shape[0]), (255, 0, 0), 3)
        cv2.line(cv_image, (border_white, 0), (border_white, cv_image.shape[0]), (0, 255, 0), 3)

        target_top = int(border_yellow + (border_white - border_yellow) / 2)
        target_bottom = int(cv_image.shape[1] / 2)

        cv2.line(cv_image, (target_bottom, cv_image.shape[0]), (target_top, 0), [0, 0, 255], 3)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        tg = cv_image.shape[0] / (target_bottom - target_top) if target_bottom - target_top != 0 else 0
        angle = math.atan(tg) if tg else math.pi/2

        compensation = (math.pi/2) - abs(angle)
        if angle < 0:
            compensation = -compensation

        # compensation = math.degrees(compensation_r)
        # self.logger.info(f'tg = {tg}, angle_r = {angle_r}, angle = {angle}, compensation_r={compensation_r}, compensation={compensation}')

        return compensation

        self.compensations.put(compensation)
        if self.compensations.full():
            return self.compensations.get()
        else:
            return None

    def detect_lines(self, image):
        region_interest = image[int(image.shape[0] * 3 / 4):,:,:]
        img_hsv = cv2.cvtColor(region_interest, cv2.COLOR_BGR2HSV)

        mask_yellow = self.get_mask(img_hsv, np.asarray([20, 100, 100]), np.asarray([30, 255, 255]))
        mask_white = self.get_mask(img_hsv, np.asarray([0, 0, 235]), np.asarray([255, 20, 255]))

        mask_yellow, border_yellow = self.get_bounding_rect(mask_yellow, is_right=False)
        mask_white, border_white = self.get_bounding_rect(mask_white, is_right=True)
        cv2.line(mask_yellow, (border_yellow, 0), (border_yellow, mask_yellow.shape[0]), color=[255, 255, 255], thickness=3)
        cv2.line(mask_white, (border_white, 0), (border_white, mask_white.shape[0]), color=[255, 255, 255], thickness=3)

        # cv2.imshow('yellow mask', mask_yellow)
        # cv2.imshow('white mask', mask_white)

        return border_yellow, border_white

    def get_mask(self, img_hsv, color_low, color_hight):
        return cv2.inRange(img_hsv, color_low, color_hight)

    def get_bounding_rect(self, mask, is_right=True):
        '''
        Returns border x coordinate of a last bounding rectangle from right or left
        :param mask: black/white mask
        :param is_right: should we return left border of a last right rectangle (True) or a right border of a last left rectangle (False)
        :return: tuple of mask image and found border x position
        '''
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_rects = 0, 0, 0, 0
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h > max_rects[2] * max_rects[3]:
                max_rects = x, y, w, h
            if w > 5 and h > 10:
                cv2.rectangle(mask, (x, y), (x + w, y + h), (155, 155, 0), 1)
        if max_rects[2] * max_rects[3] > 50:
            border_x = max_rects[0] if is_right else max_rects[0] + max_rects[2]
        else:
            border_x = mask.shape[1] if is_right else -1
        return mask, border_x
