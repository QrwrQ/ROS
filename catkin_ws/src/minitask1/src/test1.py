#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist

# Square is not very accurately mapped, someone could look at this and improve

def driver():

    # Inititalise publisher and node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('squareDriver', anonymous=True)

    # Initialise current time and set the rate to 10Hz
    rate = rospy.Rate(10)

    # Create twist message
    twistMessage = Twist()

    while not rospy.is_shutdown():

        time = rospy.Time.now().to_sec()

        # Move forward for 10 secs at 0.1m/s
        while rospy.Time.now().to_sec() - time < rospy.Duration(10).to_sec():
            rospy.loginfo("Moving Forward")
            twistMessage.linear.x = 0.1
            twistMessage.angular.z = 0.0
            pub.publish(twistMessage)
            rate.sleep()

        time = rospy.Time.now().to_sec()

        # Rotate for 5 secconds at ((math.pi)/2/5)rads/s. (pi/2/duration)
        while rospy.Time.now().to_sec() - time < rospy.Duration(5).to_sec():
            rospy.loginfo("Turning 90 degrees")
            twistMessage.linear.x = 0.0
            twistMessage.angular.z = (math.pi)/2/5
            pub.publish(twistMessage)
            rate.sleep()

        time = rospy.Time.now().to_sec()

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass

