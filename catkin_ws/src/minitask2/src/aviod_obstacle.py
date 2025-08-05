#!/usr/bin/env python

import rospy
import tf
import math
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time


def normalise_angle(a):
    while a > 2*math.pi:
        a -= 2*math.pi
    while a < 0:
        a += 2*math.pi
    return a

def dist(pos1, pos2):
    return math.sqrt(math.pow(pos1.x - pos2.x, 2) + math.pow(pos1.y - pos2.y, 2))

def angle_from_to(base_x, base_y, x, y):
    delta_x = x - base_x
    delta_y = y - base_y
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

class Position:
    def __init__(self, x=0, y=0, yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw


class Distance:
    def __init__(self, value=0):
        self.value = value
        self.judge =False        #if there is a obstcle the judge value become 1
        self.detect_d=0.5       #the distance of detecting the obstacle

    def set_dist(self, dists, idx, offset):
        dists2 = dists + dists
        a = int((idx-int(offset))) + len(dists)
        b = int((idx+int(offset))) + len(dists)
        self.value = min(dists2[a:b])
    def set_front(self,dists):    #used for judge the obstacle in front
        self.judge=0
        for i in dists[0:44]:
            if i<self.detect_d:
                self.judge=True
                return
        for i in dists[316:359]:
            if i<self.detect_d:
                self.judge=1
                return

class Distances:
    front = Distance()
    left = Distance()
    back = Distance()
    right = Distance()


class TurtlebotDriving:
    move_speed = 0.05
    turn_speed = 0.1
    front_view=0 
    activity_range=10 #give a range to the robot
    pos = Position()
    dists = Distances()
    front_v=Distance()

    # The field of view of the robot in each direction
    fov = 10 # 10 degrees (5 each side)

    cmd_vel_msg = Twist()

    def log_pose(self):
        rospy.loginfo("(%f, %f)", self.pos.x, self.pos.y)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        quaternion = [orientation.x, orientation.y, orientation.z,
                      orientation.w]
        (roll, pitch, yaw) = \
            tf.transformations.euler_from_quaternion(quaternion)
        self.pos.yaw = yaw
        self.pos.x   = pos.x
        self.pos.y   = pos.y

    def scan_callback(self, msg):
        print(msg)
        self.dists.front.set_dist(msg.ranges,   0, self.fov/2)
        self.dists.right.set_dist(msg.ranges, 270, self.fov/2)
        self.dists.back.set_dist( msg.ranges, 180, self.fov/2)
        self.dists.left.set_dist( msg.ranges,  90, self.fov/2)

    def r_scan_callback(self,msg):  # aim at the front, rossi's version listennner's callback
        print("the distance of the obstacle:"+str(self.front_view))
        print(msg)
        self.front_view=msg.ranges[0]
        
        self.front_v.set_front(msg.ranges)

    def move(self, speed):
        self.cmd_vel_msg.linear.x = speed
        self.cmd_vel_msg.angular.z = 0

    def turn(self, speed):
        self.cmd_vel_msg.angular.z = speed
        self.cmd_vel_msg.linear.x= 0

    def update_commands(self):
        self.pub.publish(self.cmd_vel_msg)

    def stop(self):
        self.cmd_vel_msg = Twist()
        self.update_commands()

    def avoid_obstacles(self):
        if self.range_front < 0.5:
            rospy.loginfo("Obstacle in front! Rotating until no obstacle...")
            self.stop()
            while self.range_front < 0.5:
                self.turn(self.turn_speed)
                self.rate.sleep()
                
    def r_avoid_obstacles(self):    #rosii's version avoid the obstacle
        time.sleep(2)
        print("the distance of the obstacle:"+str(self.front_view))
        while self.pos.x+self.pos.y<10:    #set the range of activity around 10
            #if self.front_view < 0.5:
            if self.front_v.judge==True:
                rospy.loginfo("Obstacle in front! Rotating until no obstacle...")
                self.turn(self.turn_speed)
                self.update_commands()
                self.rate.sleep()
            #if self.front_view >0.5:
            if self.front_v.judge==False:
                self.move(self.move_speed)
                rospy.loginfo("cannot touch me!")
                self.rate.sleep()
                self.update_commands()
            time.sleep(1)
        while self.pos.x+self.pos.y>10:    #if the robot beyond the range then stop
            print("I'm stopping")
            self.stop()
            self.update_commands()
            self.rate.sleep()
            '''
            while self.range_front < 0.5:
                self.turn(self.turn_speed)
                self.rate.sleep()
            '''
    def follow_rh_wall(self):
        pass

    def random_walk(self):
        start = self.pos
        self.move(self.move_speed)
        while dist(start, self.pos) < 3:
            self.avoid_obstacles()
            self.rate.sleep()
        # TODO: turn randomly

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.scan = rospy.Subscriber('scan', LaserScan, self.r_scan_callback)
        rospy.init_node('driver')
        self.rate = rospy.Rate(10)
        #rospy.Timer(rospy.Duration(0.1), self.update_commands)


if __name__ == '__main__':
    driver = TurtlebotDriving()
    try:
        driver.r_avoid_obstacles()
        rospy.spin()
    except KeyboardInterrupt:
        driver.stop()

