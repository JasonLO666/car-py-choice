#!/usr/bin/env python
#coding=utf-8
import tf
import rospy
import time
import thread
from robot_msg.msg import RoboState
from std_msgs.msg import String,UInt8
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import Odometry
from uid_provider import UID
from pos_cache import Pos_Cache
class State_Center:
    def __init__(self):
        #Cache初始化
        uid_file=rospy.get_param('~uid_filename','/home/zhangdeyu/clean_v6/src/cfg/param/uid.yaml')
        cache_file=rospy.get_param('~pos_cache_file','/home/zhangdeyu/clean_v6/src/cfg/cache.yaml')
        self.cache=RoboState()
        self.cache.UID,self.cache.type,self.cache.key=UID(uid_file)
        self.cache.job_state=''
        self.file_cache=Pos_Cache(cache_file)
        self.file_cache.Scene_Init()
        #接收回调函数
        self.bat_Sub=rospy.Subscriber('battery',UInt8,callback=self.BAT)
        self.utils=rospy.Subscriber('utils_real',UInt8,callback=self.UTILS)
        self.scene_Sub=rospy.Subscriber('scene_str',String,callback=self.SCENE)
        self.state_Pub=rospy.Publisher('robot_state',RoboState,queue_size=5,latch=True)
    def Activate(self):
        thread.start_new_thread(self.Thread_Refresh,())
    def Thread_Refresh(self):
        '''
            独立线程获取坐标变换信息
        '''
        listener=tf.TransformListener()
        while not rospy.is_shutdown():
            time.sleep(0.02)
            if self.cache.job_state=='SLEEP':
                self.cache.state='SLEEP'
                self.state_Pub.publish(self.cache)
                continue
            try:
                (trans,rot)=listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                (roll,pitch,self.cache.ang)=tf.transformations.euler_from_quaternion(rot)
                self.cache.x=trans[0]
                self.cache.y=trans[1]
                self.cache.state='OK'
                self.state_Pub.publish(self.cache)
                self.file_cache.Pos_Refresh(x=self.cache.x,\
                                            y=self.cache.y,\
                                            ang=self.cache.ang,\
                                            scene=self.cache.area)
                

            except Exception,err:
                self.cache.state='TF_ERROR'
                self.state_Pub.publish(self.cache)
                #rospy.logerr(err)
    #回调函数
    def BAT(self,data):
        self.cache.battery=data.data
        #此处加入电压处理程序
    def UTILS(self,data):
        self.cache.utils=str(data.data)
        #外设控制
        if data.data==5: self.cache.utils='up'
        elif data.data==10: self.cache.utils='down'
        else: self.cache.utils='unknow'
    def SCENE(self,data):
        self.cache.area=data.data
        self.file_cache.Pos_Refresh(x=self.cache.x,\
                                            y=self.cache.y,\
                                            ang=self.cache.ang,\
                                            scene=self.cache.area,\
                                            forced=True)
if __name__=='__main__':
    rospy.init_node('robot_state_center')
    s=State_Center()
    s.Activate()
    rospy.spin()