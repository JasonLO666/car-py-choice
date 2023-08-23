#coding=utf-8
#!/usr/bin/env python
#coding=utf-8
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped as Pose
def Get_TF(parent='/map',child='/base_link',listener=None):
    try:
        time.sleep(0.01)
        (trans,rot)=listener.lookupTransform(parent,child, rospy.Time(0))
        roll,pitch,yaw=tf.transformations.euler_from_quaternion(rot)
        return trans[0],trans[1],yaw
    except Exception,err:
        rospy.logerr(err)
        return [0,0,0]
def Get_TF_Stamped(parent='/map',child='/base_link',listener=None):
    while not rospy.is_shutdown():
        try:
            time.sleep(0.01)
            trans,rot=listener.lookupTransform(parent,child, rospy.Time(0))
            p=Pose()
            p.header.stamp=rospy.Time.now()
            p.header.seq+=1
            p.header.frame_id='map'
            p.pose.position.x=trans[0]
            p.pose.position.y=trans[1]
            p.pose.position.z=trans[2]
            p.pose.orientation.x=rot[0]
            p.pose.orientation.y=rot[1]
            p.pose.orientation.z=rot[2]
            p.pose.orientation.w=rot[3]
            return p
        except Exception,err:
            pass
            #print err
