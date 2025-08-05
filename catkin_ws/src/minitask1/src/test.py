#!/usr/bin/env python

import rospy
import math 
from geometry_msgs.msg import Twist

def test():

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('testcom', anonymous=True)
    rate= rospy.Rate(100)
    com1=Twist()
    rospy.loginfo("keeping")
    pub.publish(com1)
    rate.sleep()
    while not rospy.is_shutdown():
        time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - time < rospy.Duration(10).to_sec():
            rospy.loginfo("Moving Forward")
            com1.linear.x=0.1
            com1.angular.z=0.0
            pub.publish(com1)
            rate.sleep()
        time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - time < rospy.Duration(10).to_sec():
            rospy.loginfo("turn")
            com1.linear.x=0.0
            com1.angular.z=math.pi/20
            pub.publish(com1)
            rate.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
