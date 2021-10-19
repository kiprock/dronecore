#!/usr/bin/env python

import rospy
import tf2_geometry_msgs
import tf
import math
from tf.transformations import euler_from_quaternion


def pose_republish(posedata):

    #do the transformation
    global x1,y1,z1,yaw1,pitch1,roll1
    x1=posedata.pose.position.x*-1
    y1=posedata.pose.position.y
    posedata.pose.position.x=y1
    posedata.pose.position.y=x1
    
    euler = euler_from_quaternion([posedata.pose.orientation.x, posedata.pose.orientation.y, posedata.pose.orientation.z, posedata.pose.orientation.w]) 
    roll = euler[1]
    pitch = -1*(euler[0]+1.57)
    yaw = euler[2]
    new_angle = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    posedata.pose.orientation.x=new_angle[0]
    posedata.pose.orientation.y=new_angle[1]
    posedata.pose.orientation.z=new_angle[2]
    posedata.pose.orientation.w=new_angle[3]

    pub = rospy.Publisher('/mavros/vision_pose/pose', tf2_geometry_msgs.PoseStamped, queue_size=1.0)
    pub.publish(posedata)

if __name__ == '__main__':

    rospy.init_node('republisher', anonymous=False)
    rospy.Subscriber("/slamcore/pose", tf2_geometry_msgs.PoseStamped, pose_republish)
    rospy.spin()
