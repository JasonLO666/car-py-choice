#!/usr/bin/env python
#!coding=utf-8
 
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys
 
 
def webcamImagePub():
    # init ros_node
    rospy.init_node('cam')
    # queue_size should be small in order to make it 'real_time'
    # or the node will pub the past_frame
    img_pub = rospy.Publisher('/cam/monitor', Image, queue_size=2)
    rate = rospy.Rate(5) # 5hz 
    cap = cv2.VideoCapture(0)
    scaling_factor = 0.5
    bridge = CvBridge()
    if not cap.isOpened():
        rospy.logerr('camera initializing falied')
        return -1
    count = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            #frame = cv2.resize(frame,None,fx=scaling_factor,fy=scaling_factor,interpolation=cv2.INTER_AREA)
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_pub.publish(msg)
        else:
            rospy.loginfo("Capturing image failed.")
        rate.sleep()
 
if __name__ == '__main__':
    try:
        webcamImagePub()
    except rospy.ROSInterruptException:
        pass