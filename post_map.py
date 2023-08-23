#!/usr/bin/env python
#coding=utf-8
import rospy 
import numpy as np
import cv2
from nav_msgs.srv import GetMap,GetMapRequest,GetMapResponse
from nav_msgs.msg import OccupancyGrid as Map
from robot_msg.srv import *
from std_msgs.msg import String
from http_uploader import Http2Server
class Map_Uploader:
    def __init__(self):
        self.postBridge=rospy.Service('post_img',post_img,self.Post)
        self.reScene=rospy.Publisher('scene_str',String,latch=True,queue_size=1)
        self.map=None
        self.map_Recv=rospy.Subscriber('/map',Map,callback=self.map_Cb)
        self.uploader=Http2Server()
    def Post(self,req):
        name=req.map_name
        map=self.map
        height=map.info.height
        width=map.info.width
        data=np.zeros((height,width,3),dtype=np.int16)
        for h in range(height):
            data[h,:,0]=map.data[h*width:(h+1)*width]
        data[:,:,1]=data[:,:,2]=data[:,:,0]
        data[data>0]=1
        data[data==-1]=205
        data[data==0]=254
        data[data==1]=0
        img_jpeg=cv2.imencode('.jpg',data)[1]
        encoded = np.array(img_jpeg)
        data=encoded.tostring()
        ret= self.uploader.Upload(data,name)
        self.reScene.publish(name)
        return ret
    def map_Cb(self,dat):
        self.map=dat
if __name__=='__main__':
    rospy.init_node('map_uploader')
    up=Map_Uploader()
    rospy.spin()

