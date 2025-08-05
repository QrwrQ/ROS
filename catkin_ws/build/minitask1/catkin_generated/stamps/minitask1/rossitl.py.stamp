import rospy
import geometry_msgs.msg import Twist

def open_loop():
    pub = rospy.Publisher('/cmd_vel', geometry_msgs, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    r = rospy.Rate(10)
    t = rospy.Time.now().to_sec()
    msgs1=Twist()
    mags2=msgs1
    msgs.linear.x=1
    msgs.angular.z=1

    while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec():
        pub.publish(msgs1)
        pub.publish(msgs2)
        r.sleep()
if __name__ == '__main__':
    try:
        open_loop()
     except rospy.ROSInterruptException:
        pass
