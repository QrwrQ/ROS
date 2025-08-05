#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MySubscriber:

    def __init__(self):
        self.sub = rospy.Subscriber('odom', Odometry, self.myCallback)
        self.pub = rospy.Publisher('warning', String, queue_size=10)

    def myCallback(self, msg):
        if msg.twist.twist.angular.z < 1.2:
            alert = "Angular velocity too high!"
            print(alert)
            s = String()
            s.data = alert
            self.pub.publish(s)

rospy.init_node("myFirstSubscriber",anonymous=True)
fp = MySubscriber()
rospy.spin()
