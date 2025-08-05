#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
num_secs=4

def open_loop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    r = rospy.Rate(10)
    t = rospy.Time.now().to_sec()
    #msgs1=Twist((1,0,0),(0,0,0))
    #msgs2=Twist((0,0,0),(0,0,1))
    msgs1=Twist()
    msgs2=Twist()
    msgs1.linear.x=0.05
    msgs2.angular.z=1
    #rospy.loginfo("Test")
    #pub.publish(msgs1)
    #r.sleep()
    i=0
    for i in range(10):
        print("testing")
        pub.publish(msgs1)
        #r.sleep()
    '''
    while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec():
        rospy.loginfo("Test")
        pub.publish(msgs1)
    '''
        #pub.publish(msgs2)
       # r.sleep()

if __name__ == '__main__':
    try:
        open_loop()
    except rospy.ROSInterruptException:
        pass
