#!/usr/bin/env python

import rospy
import tf
import math
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalise_angle(a):
    while a > 2*math.pi:
        a -= 2*math.pi
    while a < 0:
        a += 2*math.pi
    return a

def euclidean_dist(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))

def angle_from_to(base_x, base_y, x, y):
    delta_x = x - base_x
    delta_y = y - base_y
    #return math.atan(delta_y / delta_x)
    theta = math.atan(delta_y / delta_x)
    if delta_x >= 0 and delta_y >= 0:
        return theta
    elif delta_x < 0 and delta_y >= 0:
        return math.pi + theta
    elif delta_x < 0 and delta_y < 0:
        return theta - math.pi
    else:
        return theta


# The angular distance from a to b
def turn_amount(a, b):
    a = normalise_angle(a)
    b = normalise_angle(b)
    delta = b - a + 3/2*math.pi
    while delta > 2*math.pi:
        delta -= 2*math.pi
    delta -= math.pi
    return delta


class TurtlebotDriving:
    desired_yaw = 0
    yaw = 0
    x = 0
    y = 0
    cmd_vel_msg = Twist()
    move_speed = 0
    turn_speed = 0

    min_speed = 0.01
    max_speed = 0.2
    min_turn_speed = 0.01
    max_turn_speed = 0.2

    angle_threshold = 0.001
    linear_threshold = 0.001

    def log_pose(self):
        rospy.loginfo("(%f, %f))" % (self.x, self.y))

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        quaternion = [orientation.x, orientation.y, orientation.z,
                      orientation.w]
        (roll, pitch, yaw) = \
            tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = yaw
        self.x = pos.x
        self.y = pos.y
        #self.log_pose()

    def move(self, speed):
        self.move_speed = speed
        self.cmd_vel_msg.linear.x = speed

    def distance_to(self, x, y):
        return euclidean_dist(self.x, self.y, x, y)

    def move_dist(self, d=1):
        start_x = self.x
        start_y = self.y
        distance = self.distance_to(start_x, start_y)
        while distance < d:
            speed1 = distance / d * self.max_speed
            speed1 = min(max(self.min_speed, speed1), self.max_speed)
            speed2 = (d - distance) / d * self.max_speed
            speed2 = min(max(self.min_speed, speed2), self.max_speed)
            speed = min(speed1, speed2)
            self.move(0.1)
            self.rate.sleep()
            distance = self.distance_to(start_x, start_y)
        self.stop()

    def turn_angle(self, speed, a=math.pi/2):
        start_yaw = self.yaw
        self.desired_yaw = self.yaw + a
        while self.heading_error() > self.angle_threshold:
            self.turn_towards_heading()
            self.rate.sleep()
        self.stop()

    def turn(self, speed):
        self.turn_speed = speed
        self.cmd_vel_msg.angular.z = speed

    def heading_error(self):
        return abs(turn_amount(self.yaw, self.desired_yaw))

    def turn_to(self, a):
        self.desired_yaw = a
        while self.heading_error() > self.angle_threshold:
            self.turn_towards_heading()
        self.stop()

    def move_to(self, target_x, target_y):
        #rospy.loginfo("Moving to (%f, %f)..." % (target_x, target_y))
        self.rate.sleep()
        max_dist = euclidean_dist(self.x, self.y, target_x, target_y)
        distance = max_dist
        #rospy.loginfo("Distance: %f" % distance)
        if (max_dist < self.linear_threshold):
            return
        # Calculate required yaw
        self.desired_yaw = angle_from_to(self.x, self.y, target_x, target_y)
        #rospy.loginfo("Desired yaw: %f" % self.desired_yaw)
        # Turn to the target
        while abs(self.desired_yaw - self.yaw) > self.angle_threshold:
            self.turn_towards_heading()
            self.rate.sleep()
        while (distance > self.linear_threshold):
            # Calculate required yaw
            self.desired_yaw = angle_from_to(self.x, self.y,
                                             target_x, target_y)
            # Turn to the target
            self.turn_towards_heading()
            # Start moving
            distance = euclidean_dist(self.x, self.y, target_x, target_y)
            max_dist = max(max_dist, distance)
            speed = distance / max_dist * max_speed
            speed = min(max(self.min_speed, speed), self.max_speed)
            self.move(speed)
            self.rate.sleep()
        # Stop moving
        self.stop()

    def odom_listener(self):
        rospy.Subscriber('odom', Odometry, self.odom_callback)

    def turn_towards_heading(self):
        delta = turn_amount(self.yaw, self.desired_yaw)
        self.turn(delta)

    def update_commands(self, event):
        #self.turn_towards_heading()
        self.pub.publish(self.cmd_vel_msg)

    def spin(self):
        rospy.Timer(rospy.Duration(0.1), self.update_commands)

    def stop(self):
        #rospy.loginfo("Stopping.")
        self.cmd_vel_msg = Twist()
        self.pub.publish(Twist())

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('driver')
        self.rate = rospy.Rate(10)
        self.odom_listener()
        self.spin()


if __name__ == '__main__':
    driver = TurtlebotDriving()
    try:
        driver.move_to(0, 0)
        driver.log_pose()
        driver.turn_to(0)
        driver.move_dist()
        driver.log_pose()
        driver.turn_angle(0.1)
        driver.move_dist()
        driver.turn_angle(0.1)
        driver.log_pose()
        driver.move_dist()
        driver.turn_angle(0.1)
        driver.log_pose()
        driver.move_dist()
        driver.log_pose()
    except KeyboardInterrupt:
        driver.stop()

