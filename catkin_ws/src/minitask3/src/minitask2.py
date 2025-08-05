#!/usr/bin/env python

import rospy
import tf
import math
import random
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# Returns distances between two points
def dist(pos1, pos2):
    return math.sqrt(math.pow(pos1.x - pos2.x, 2) +
                     math.pow(pos1.y - pos2.y, 2))


# Overly complicated - TODO: use a dictionary instead?
class Position:
    def __init__(self, x=0, y=0, yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def copy(self):
        return Position(self.x, self.y, self.yaw)


class Distance:
    def __init__(self, value=0):
        self.value = value

    # Finds and sets the minimum range value in the fov range for a given
    # direction.
    # dists is the list of distances
    # idx is the center index for the current direction
    # offset is half of the fov (for this direction)
    def set_dist(self, dists, idx, offset):
        # A double length list (used so we can wrap around the beginning/end
        # easily)
        dists2 = dists + dists
        # a is minimum index for fov range while b is max index
        a = int((idx-int(offset))) + len(dists)
        b = int((idx+int(offset))) + len(dists)

        # sets self.value to be minimum value within the range set by a and b
        self.value = min(dists2[a:b])

    def get_dist(self):
        # Returns the value stored at self.value
        return self.value


class TurtlebotDriving:
    move_speed = 0.1
    turn_speed = 0.1

    front_obstacle_dist = 0.5
    fl_obstacle_dist = 0.25
    fr_obstacle_dist = 0.25

    pos = Position()
    dists = {'front': Distance(),
             'fl': Distance(),
             'left': Distance(),
             'back': Distance(),
             'rb': Distance(),
             'right': Distance(),
             'rf': Distance(),
             'fr': Distance(),
             'wfr': Distance(),
             'wbr': Distance()}

    # Half of the field of view of the robot in each direction
    fov = {'front': 10,
           'fl': 5,
           'left': 10,
           'back': 10,
           'rb': 1,
           'right': 10,
           'rf': 1,
           'fr': 5,
           'wfr': 1,
           'wbr': 1}

    # Angles of laser scans for each direction
    angles = {'front': 0,
              'fl': 45,
              'left': 90,
              'back': 180,
              'rb': 255,
              'right': 270,
              'rf': 285,
              'fr': 315,
              'wfr': 330,
              'wbr': 210}

    # Avoid obstacles definitions
    ao_in_progress = False
    ao_turn_speed = None
    ao_start_speed = None
    ao_view_range = 2

    # Random walk definitions
    random_walk_dist = 3
    random_turn_min_time = 1
    random_turn_max_time = 10
    rwalk_start = None
    rwalk_in_progress = False
    rturn_in_progress = False
    rturn_end_time = None
    rturn_speed = None

    # Follow RH wall definitions
    frhw_in_progress = False
    frhw_start_right_dist = None
    frhw_target_dist = 0.5

    # Twist message for controlling the robot
    cmd_vel_msg = Twist()

    def log_pose(self):
        rospy.loginfo("(%f, %f)", self.pos.x, self.pos.y)

    def current_speed(self):
        return self.cmd_vel_msg.linear.x

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        quaternion = [orientation.x, orientation.y, orientation.z,
                      orientation.w]
        (roll, pitch, yaw) = \
            tf.transformations.euler_from_quaternion(quaternion)
        self.pos.yaw = yaw
        self.pos.x = pos.x
        self.pos.y = pos.y

    def scan_callback(self, msg):
        self.dists['front'].set_dist(msg.ranges, self.angles['front'],
                                     self.fov['front'])
        self.dists['fl'].set_dist(msg.ranges, self.angles['fl'],
                                  self.fov['fl'])
        self.dists['left'].set_dist(msg.ranges, self.angles['left'],
                                    self.fov['left'])
        self.dists['back'].set_dist(msg.ranges, self.angles['back'],
                                    self.fov['back'])
        self.dists['rb'].set_dist(msg.ranges, self.angles['rb'],
                                  self.fov['rb'])
        self.dists['right'].set_dist(msg.ranges, self.angles['right'],
                                     self.fov['right'])
        self.dists['rf'].set_dist(msg.ranges, self.angles['rf'],
                                  self.fov['rf'])
        self.dists['fr'].set_dist(msg.ranges, self.angles['fr'],
                                  self.fov['fr'])
        self.dists['wfr'].set_dist(msg.ranges, self.angles['wfr'],
                                   self.fov['wfr'])
        self.dists['wbr'].set_dist(msg.ranges, self.angles['wbr'],
                                   self.fov['wbr'])

    def move(self, speed):
        self.cmd_vel_msg.linear.x = speed

    def turn(self, speed):
        self.cmd_vel_msg.angular.z = speed

    def stop(self):
        self.cmd_vel_msg = Twist()
        self.pub.publish(self.cmd_vel_msg)

    def sees_obstacles(self):
        # Do we need to turn to avoid obstacles?
        if self.dists['front'].get_dist() < self.front_obstacle_dist or \
                self.dists['fl'].get_dist() < self.fl_obstacle_dist or \
                self.dists['fr'].get_dist() < self.fr_obstacle_dist:
            return True
        return False

    def avoid_obstacles_start(self):
        self.ao_in_progress = True
        self.ao_start_speed = self.current_speed()
        # Do we need to turn one specific direction?
        turn_right = False
        turn_left = False
        if self.dists['fl'].get_dist() < self.fl_obstacle_dist:
            turn_right = True
        if self.dists['fr'].get_dist() < self.fr_obstacle_dist:
            turn_left = True
        self.ao_turn_speed = random.choice([-1, 1]) * self.turn_speed
        self.stop()
        if turn_left and not turn_right:
            rospy.loginfo("Obstacle front-right (dist %.2f)!",
                          self.dists['fr'].get_dist())
            self.ao_turn_speed = abs(self.ao_turn_speed)
        elif turn_right and not turn_left:
            rospy.loginfo("Obstacle front-left (dist %.2f)!",
                          self.dists['fl'].get_dist())
            self.ao_turn_speed = -abs(self.ao_turn_speed)
        else:
            rospy.loginfo("Obstacle in front (dist %.2f)!",
                          self.dists['front'].get_dist())

    def avoid_obstacles(self):
        if not self.ao_in_progress:
            self.avoid_obstacles_start()
        else:
            self.turn(self.ao_turn_speed)

    def cancel_avoid_obstacles(self):
        if self.ao_in_progress:
            self.stop()
            self.move(self.ao_start_speed)
            self.ao_in_progress = False
            self.ao_turn_speed = None
            self.ao_start_speed = None

    def sees_rh_wall(self):
        return ((self.dists['right'].get_dist() < 0.5) and
                (self.dists['wfr'].get_dist() < 1) and
                (self.dists['wbr'].get_dist() < 1))

    def start_follow_rhw(self):
        self.frhw_in_progress = True
        self.frhw_start_right_dist = self.dists['right'].get_dist()

    # Align ourselves parallel to the wall
    def follow_rhw_check_parallel(self):
        if self.dists['rf'].get_dist() < self.dists['right'].get_dist():
            self.turn(-self.turn_speed)
            return False
        elif self.dists['rb'].get_dist() < self.dists['right'].get_dist():
            self.turn(self.turn_speed)
            return False
        self.turn(0)
        return True

    # Maintain our distance from the wall and follow it
    def move_follow_rhw(self):
        # Maintain distance from RH wall
        if self.dists['right'].get_dist() < self.frhw_target_dist-0.1:
            self.turn(self.turn_speed)  # Turn left
        elif self.dists['right'].get_dist() > self.frhw_target_dist+0.1:
            self.turn(-self.turn_speed)  # Turn right
        else:
            self.turn(0)
            self.move(self.move_speed)

    def follow_rh_wall(self):
        if not self.frhw_in_progress:
            self.start_follow_rhw()
        if not self.follow_rhw_check_parallel():
            return
        else:
            self.move_follow_rhw()

    def cancel_follow_rh_wall(self):
        if self.frhw_in_progress:
            self.frhw_in_progress = False
            self.frhw_start_right_dist = None

    # Start a random walk
    def start_rwalk(self):
        self.rwalk_start = self.pos.copy()
        self.rwalk_in_progress = True

    # Are we currently in the middle of walking randomly?
    def is_rwalking(self):
        return self.rwalk_in_progress or self.rturn_in_progress

    # Continue an ongoing random walk
    def continue_rwalk(self):
        if self.rwalk_in_progress and self.continue_rwalk_move():
            return True
        if self.rturn_in_progress:
            return self.continue_rwalk_turn()

    # Continue an rwalk in the state of moving
    def continue_rwalk_move(self):
        if dist(self.rwalk_start, self.pos) < self.random_walk_dist:
            # Continue moving
            self.turn(0)
            self.move(self.move_speed)
            return True
        else:
            # Moved far enough, start turning
            self.move(0)
            self.rwalk_start = None
            self.rwalk_in_progress = False
            self.rturn_in_progress = True
            # Setup turn parameters
            rturn_start = rospy.Time.now()
            rturn_duration = rospy.Duration(
                    random.uniform(self.random_turn_min_time,
                                   self.random_turn_max_time))
            self.rturn_end_time = rturn_start + rturn_duration
            self.rturn_speed = self.turn_speed * random.choice([-1, 1])
            return False

    # Continue an rwalk in the state of turning
    def continue_rwalk_turn(self):
        if rospy.Time.now() < self.rturn_end_time:
            self.move(0)
            self.turn(self.rturn_speed)
            return True
        else:
            self.turn(0)
            self.rturn_in_progress = False
            self.rturn_end_time = None
            self.rturn_speed = None
            return False

    # Cancel an rwalk
    def cancel_rwalk(self):
        if self.is_rwalking():
            self.stop()
            self.rwalk_in_progress = False
            self.rwalk_start = None
            self.rturn_in_progress = False
            self.rturn_end_time = None
            self.rturn_speed = None

    # Start or continue walking randomly
    def random_walk(self):
        if not self.is_rwalking():
            self.start_rwalk()
        else:
            self.continue_rwalk()

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.scan = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        if image == True
            self.scan=rospy.Subscriber()
        rospy.init_node('driver')
        rospy.Timer(rospy.Duration(0.1),
                    lambda ev: self.pub.publish(self.cmd_vel_msg))


# A class for controlling the behaviours of the robot
class TurtlebotController:
    state = ' '
    state_previous = ' '
    state_changed = False

    def decide_behaviour(self):
        self.state_changed = False
        if self.driver.sees_obstacles():
            if self.state != 'a':
                self.state_changed = True
                self.state_previous = self.state
                rospy.loginfo("See obstacles! Avoiding them...")
            self.state = 'a'  # Avoiding obstacles
        # elif self.driver.sees_rh_wall():
        #     if self.state != 'f':
        #         self.state_changed = True
        #         self.state_previous = self.state
        #         rospy.loginfo("See RH wall! following it...")
        #     self.state = 'f'  # Following wall
        else:
            if self.state != 'r':
                self.state_changed = True
                self.state_previous = self.state
                rospy.loginfo("Don't see anything! Walking randomly...")
            self.state = 'r'  # Randomly walking

    def enact_behaviour(self):
        if self.state_changed:
            if self.state_previous == 'r':
                self.driver.cancel_rwalk()
            elif self.state_previous == 'f':
                self.driver.cancel_follow_rh_wall()
            elif self.state_previous == 'a':
                self.driver.cancel_avoid_obstacles()

        if self.state == 'a':
            self.driver.avoid_obstacles()
        elif self.state == 'f':
            self.driver.follow_rh_wall()
        else:
            self.driver.random_walk()

    def behaviour_loop(self):
        while True:
            self.decide_behaviour()
            self.enact_behaviour()
            self.rate.sleep()

    def __init__(self):
        self.driver = TurtlebotDriving()
        self.rate = rospy.Rate(10)
        # Wait until we have data from the laser scanner
        while (self.driver.dists['front'].get_dist() == 0):
            self.rate.sleep()


if __name__ == '__main__':
    controller = TurtlebotController()
    try:
        controller.behaviour_loop()
    except KeyboardInterrupt:
        controller.driver.stop()
