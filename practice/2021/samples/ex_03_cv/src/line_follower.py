#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

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
            border_x = -1
        return mask, border_x


    def detect_lines(self, image):
        region_interest = image[int(image.shape[0] * 3 / 4):,:,:]
        cv2.imshow('interest', region_interest)
        cv2.waitKey(3)
        img_hsv = cv2.cvtColor(region_interest, cv2.COLOR_BGR2HSV)
        mask_yellow = self.get_mask(img_hsv, np.asarray([20, 100, 100]), np.asarray([30, 255, 255]))
        mask_white = self.get_mask(img_hsv, np.asarray([0, 0, 235]), np.asarray([255, 20, 255]))

        cv2.imshow('yellow mask', mask_yellow)
        cv2.imshow('white mask', mask_white)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #rospy.loginfo(f'image received {cv_image.shape}')
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        scale_percent = 30  # percent of original size
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)


        cv2.imshow("Image window", resized)
        cv2.waitKey(3)

        try:
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            pass
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)