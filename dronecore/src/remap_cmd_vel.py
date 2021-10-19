#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TwistStamped
import tf2_geometry_msgs
import tf
import math

vel_data=TwistStamped()

def cmd_vel_republish(vel_data):
    
    vel_data2=TwistStamped()
    vel_data2.header.stamp = rospy.Time.now()
    vel_data2.header.frame_id = "slamcore/map"

    vel_data2.twist.x=vel_data.twist

    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1.0)
    pub.publish(vel_data2)

if __name__ == '__main__':

    rospy.init_node('republisher', anonymous=False)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_republish)
    rospy.spin()
