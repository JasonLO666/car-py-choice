#!/usr/bin/env python
#coding=utf-8
'''pos_init ROS Node'''
import rospy
import tf
import math
import time
import genpy
import thread
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
filename=""
scene=""
linear_accurancy=0.1
angular_accurancy=0.1
def scene_Callback(data):
    global scene
    scene=data.data
'''
    刷新位置信息
'''
class Pos_Cache:
    def __init__(self,filename):
        self.cache=[0.0,0.0,0.0]
        self.file=filename
        self.scene_Pub=rospy.Publisher('scene_str',String,queue_size=5,latch=True)
        self.pos_Pub=rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=5,latch=True)
    def Pos_Refresh(self,x,y,ang,scene,forced=False):
        try:
            if (math.fabs(x-self.cache[0])<linear_accurancy) and \
                (math.fabs(y-self.cache[1])<linear_accurancy) and \
                (math.fabs(ang-self.cache[2])<angular_accurancy) and \
                (not forced): return 0
            if x==0 and y==0 and ang==0: return
            f2write = open(self.file,'w+')
            f2write.write('x: '+str(x)+'\n')
            f2write.write('y: ' + str(y) + '\n')
            f2write.write('a: ' + str(ang) + '\n')
            f2write.write('area: ' + scene)
            f2write.close()
            self.cache=[x,y,ang]
            return 0
        except Exception,err:
            return -1
    def Scene_Init(self):
        try:
            x=0
            y=0
            a=0
            area='NO MAP'
            f2read=open(self.file,'r')
            while not rospy.is_shutdown():
                line=f2read.readline()
                if not line:
                    break
                line=line.split(': ')
                if line[0] == 'area': 
                    area = line[1]
                    self.scene_Pub.publish(String(area))
                elif line[0]=='x': x=float(line[1])
                elif line[0]=='y': y=float(line[1])
                elif line[0]=='a': a=float(line[1])
            p=PoseWithCovarianceStamped()
            p.header.frame_id='map'
            p.header.seq+=1
            p.header.stamp=genpy.Time()
            p.pose.pose.position.x=x
            p.pose.pose.position.y=y
            p.pose.pose.orientation.x,\
                p.pose.pose.orientation.y,\
                p.pose.pose.orientation.z,\
                p.pose.pose.orientation.w=tf.transformations.quaternion_from_euler(0,0,a)
            self.pos_Pub.publish(p)
            f2read.close()

        except Exception,err:
            rospy.logerr('Error initializing scene %s',err)