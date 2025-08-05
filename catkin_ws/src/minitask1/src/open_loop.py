#!/usr/bin/env python

import rospy
import math 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

class open_loop:
    robot_x=0
    robot_y=0
    robot_theta=0
    def callback(self,msg):
        #print(rospy.get_caller_id()+"i head"+data.data)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
        rospy.loginfo("record")
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        print("----------------------------------------")
        #print(roll,pitch,yaw)
        self.robot_x=msg.pose.pose.position.x
        self.robpt_y=msg.pose.pose.position.y
        self.robot_theta=yaw
        print(self.robot_x,self.robpt_y,self.robot_theta)
        rate=rospy.Rate(10)
        rate.sleep()

    def listener(self):
        rospy.init_node('listener', anonymous=True)
     
        #rospy.Subscriber("odom", Odometry, self.callback)

        #rospy.Subscriber("odom", Odometry, self.callback)
        #rospy.spin()
        print("11111111111111111111111111111")
        #rospy.Subscriber("odom", Odometry, self.callback)
    def robot_square(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #rospy.init_node('commander', anonymous=True)
        rate= rospy.Rate(10)
        com1=Twist()
        rospy.loginfo("keeping")
        pub.publish(com1)
        rate.sleep()
        while not rospy.is_shutdown():
        #    self.listener()
            rospy.Subscriber("odom", Odometry, self.callback)
            r_x=self.robot_x
            r_y=self.robot_y
            #time = rospy.Time.now().to_sec()
            while self.robot_x+self.robot_y-r_x-r_y < 0.3:
                rospy.loginfo("Moving Forward")
                com1.linear.x=0.1
                com1.angular.z=0.0
                pub.publish(com1)
                rospy.Subscriber("odom", Odometry, self.callback)
          #      self.listener()
                rate.sleep()
         #   self.listener()
            rospy.Subscriber("odom", Odometry, self.callback)
            r_theta=self.robot_theta
            while self.robot_theta-r_theta< math.pi/20000:
                rospy.loginfo("turn")
                com1.linear.x=0.0
                com1.angular.z=math.pi/20
                pub.publish(com1)
                rospy.Subscriber("odom", Odometry, self.callback)
           #     self.listener()
                rate.sleep()
    def stop(self):
            
            pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            com_stop=Twist()
            i=0
            while i==0:
                pub.publish(com_stop)
                

if __name__ == '__main__':
    try:
        OL=open_loop()
        print("asda")
        OL.listener()

        #:OL.robot_square()
        OL.stop()
    except rospy.ROSInterruptException:
        pass
