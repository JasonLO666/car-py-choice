#!/usr/bin/env python
#!coding=utf-8
'''
 Created By Jiawei,Modified from Dalvik-Platform
'''
import rospy
import serial
import string
import math
import sys

from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
#from dynamic_reconfigure.server import Server

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
rospy.init_node("imu_node")
pub = rospy.Publisher('imu_data', Imu, queue_size=1)
imuMsg = Imu()
accl_linear_calib = [0.0,0.0,0.0]
vel_ang_calib     = [0.0,0.0,0.0]
orient_calib      = [0.0,0.0,0.0]
imuMsg.orientation_covariance = [
0.05, 0, 0,
0, 0.05, 0,
0, 0, 0.05
]
imuMsg.angular_velocity_covariance = [
0.15, 0 , 0,
0 , 0.15, 0,
0 , 0 , 0.15
]
imuMsg.linear_acceleration_covariance = [
0.18 , 0 , 0,
0 , 0.18, 0,
0 , 0 , 0.18
]
port=rospy.get_param('port','/dev/ttyACM0')

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port)
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")

for i in range(10):
    discard = ser.readline()
    print discard
x=0
rospy.loginfo("booting...")
while x < 100:
    line = ser.readline()
    words = string.split(line,",")    # Fields split
    print words
    print len(words)
    if(len(words)<9): continue
    x+=1
    accl_linear_calib[0]+=float(words[1])
    accl_linear_calib[1]+=float(words[2])
    accl_linear_calib[2]+=float(words[3])
    vel_ang_calib[0]+=float(words[4])
    vel_ang_calib[1]+=float(words[5])
    vel_ang_calib[2]+=float(words[6])
    orient_calib[0]+=float(words[7])
    orient_calib[1]+=float(words[8])
    orient_calib[2]+=float(words[9])
for it in range(3):
    accl_linear_calib[it]/=100
    vel_ang_calib[it]/=100
    orient_calib[it]/=100
now=rospy.get_time()
last=rospy.get_time()
yaw=0.0
rospy.loginfo("streaming started")
while not rospy.is_shutdown():
    try:
        line = ser.readline()
        words = string.split(line,",")    # Fields split
    except:
        continue
    if len(words) > 2:
        vector_x = (float(words[7])+37.0)/37.0
        vector_y = (float(words[8])-24.0)/24.0
        vector_z = math.atan2(vector_y,vector_x)
        imuMsg.linear_acceleration.x = (float(words[1])-accl_linear_calib[0])*9.8
        imuMsg.linear_acceleration.y = (float(words[2])-accl_linear_calib[1])*9.8
        imuMsg.linear_acceleration.z = (float(words[3])-accl_linear_calib[2])*9.8
        imuMsg.angular_velocity.x = (float(words[4])-vel_ang_calib[0])*0.0175
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -(float(words[5])-vel_ang_calib[1])*0.0175
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -(float(words[6])-vel_ang_calib[2])*0.0175
        now=rospy.get_time()
        yaw+=imuMsg.angular_velocity.z*(now-last)
        last=now
        q = quaternion_from_euler(0,0,yaw)
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'imu'
        imuMsg.header.seq = seq
        seq = words[0]
        pub.publish(imuMsg)
ser.close
#f.close