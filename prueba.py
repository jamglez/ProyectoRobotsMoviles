#!/usr/bin/env/ python3

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def procesamiento(image):
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    
    cv.imshow('test',cv_image)
    
    cv.waitKey(1)


if __name__ == '__main__':
    
    rospy.init_node('test')
    rospy.Subscriber('/camera/rgb/image_raw',Image,procesamiento)
    
    while True:
        a=1