#!/usr/bin/env python

import rospy
import tf
import math
import random
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2, cv_bridge
from sensor_msgs.msg import Image


# Returns distances between two points (tuples)
def dist(pos1, pos2):
    return math.sqrt(math.pow(pos1[0] - pos2[0], 2) +
                     math.pow(pos1[1] - pos2[1], 2))


# Class that drives the robot (publishes to topic /cmd_vel)
class Driver:
    move_speed = 0.2
    turn_speed = 0.1

    # The publisher
    pub = None 

    # Twist message for controlling the robot
    cmd_vel_msg = Twist()

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def current_speed(self):
        return self.cmd_vel_msg.linear.x

    def move(self, speed):
        self.cmd_vel_msg.linear.x = speed

    def turn(self, speed):
        self.cmd_vel_msg.angular.z = speed

    def stop(self):
        self.cmd_vel_msg = Twist()
        self.pub.publish(self.cmd_vel_msg)

    def start_timer(self, hz=10):
        rospy.Timer(rospy.Duration(1.0/hz),
                    lambda ev: self.pub.publish(self.cmd_vel_msg))

class Camera:
    hsv = None  # The HSV image
    bridge = None  # The opencv bridge
    video = None  # The video subscriber

    def __init__(self):
        self.video = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        image_resized = cv2.resize(image, (w/4,h/4))
        self.hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        self.hsv=self.hsv.astype(np.uint8)

    def apply_color_mask(self,clower, cupper):
        clower = np.array(clower, dtype = "uint8")
        upper = np.array(cupper, dtype = "uint8")
        mask = cv2.inRange(self.hsv, clower, upper)
        # Should we apply the mask to the image or just return the mask?
        return mask

# Class that tracks the position of the robot (subscribes to topic /odom)
class PositionTracker:
    position = {'x': -1,
                'y': -1,
                'yaw': -1}
    data_is_valid = False

    odom = None # odometry subscriber

    def __init__(self):
        self.odom = rospy.Subscriber('odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        quaternion = [orientation.x, orientation.y, orientation.z,
                      orientation.w]
        (roll, pitch, yaw) = \
            tf.transformations.euler_from_quaternion(quaternion)
        self.position['yaw'] = yaw
        self.position['x'] = pos.x
        self.position['y'] = pos.y

        # Mark the position data as valid
        if not self.data_is_valid:
            self.data_is_valid = True

    def pos(self):
        if self.data_is_valid:
            return (self.position['x'], self.position['y'])
        return None

    def yaw(self):
        if self.data_is_valid:
            return self.position['yaw']
        return None


# Class that handles laser scans (subscribes to topic /scan)
class LaserScanner:
    # The distances to track
    dists = {'front': {'dist': None, 'fov': 10, 'angle': 0},
             'fl': {'dist': None, 'fov': 5, 'angle': 45},
             'left': {'dist': None, 'fov': 0, 'angle': 90},
             'back': {'dist': None, 'fov': 0, 'angle': 180},
             'right': {'dist': None, 'fov': 0, 'angle': 270},
             'fr': {'dist': None, 'fov': 5, 'angle': 315}}

    # The subscriber object
    scan = None

    # Is the scan data valid?
    data_is_valid = False

    def __init__(self):
        self.scan = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        for d in self.dists.keys():
            self.set_dist(d, msg.ranges)
        self.data_is_valid = True

    def set_dist(self, dir_name, ranges):
        if dir_name not in self.dists:
            return 'invalid direction name'
        if self.dists[dir_name]['fov'] == 0:
            return 'not tracking this direction (FOV is 0)'
        # The angle is the index (idx) into the list of ranges
        idx = self.dists[dir_name]['angle']
        # The fov is the amount to (offset) the index to the left/right
        offset = self.dists[dir_name]['fov']
        # A double length list (used so we can wrap around the beginning/end
        # easily)
        ranges2 = ranges + ranges
        # a is minimum index for fov range while b is max index
        a = int((idx-int(offset))) + len(ranges)
        b = int((idx+int(offset))) + len(ranges)
        # sets value to be minimum value within the range set by a and b
        self.dists[dir_name]['dist'] = min(ranges2[a:b])

    # Get the distance of a direction (e.g. scanner.get_dist('front')   )
    def get_dist(self, dir_name):
        if ((self.data_is_valid) and (dir_name in self.dists)):
            return self.dists[dir_name]['dist']
        else: 
            return None


# Class that implements obstacle avoidance behaviour
class ObstacleAvoider:
    # Constants for modifying behaviour
    # How much further to scan when we've seen an obstacle
    alert_range_modifier = 2
    # Obstacle distance thresholds for each direction (how close before we
    # start avoiding)
    obstacle_dists = {'front': 0.5,
                      'fl': 0.25,
                      'fr': 0.25}

    # Variables used during behaviour
    ao_in_progress = False
    ao_turn_speed = None
    ao_start_speed = None

    driver = None # driver object
    scanner = None # scanner object

    def __init__(self, driver, scanner):
        self.driver = driver # Used to issue movement commands
        self.scanner = scanner # Used for laser scan

    def sees_obstacles(self):
        # Check every direction
        for direction in self.obstacle_dists:
            if self.sees_obstacles_dir(direction):
                return True
        return False

    def sees_obstacles_dir(self, dir_name):
        if dir_name not in self.obstacle_dists:
            return False
        dist_threshold = self.obstacle_dists[dir_name]
        if self.ao_in_progress:
            # If we are currently avoiding an obstacle, we want to recognise
            # obstacles at an extended distance so that we will turn
            # sufficiently far away that we will not simply walk back into the
            # obstacle once we are done.
            dist_threshold *= self.alert_range_modifier
        return self.scanner.get_dist(dir_name) < dist_threshold

    def avoid_obstacles_start(self):
        self.ao_in_progress = True
        self.ao_start_speed = self.driver.current_speed()
        # Do we need to turn one specific direction?
        obstacle_fl = False
        obstacle_fr = False
        if self.sees_obstacles_dir('fl'):
            obstacle_fl = True
        if self.sees_obstacles_dir('fr'):
            obstacle_fr = True
        self.ao_turn_speed = random.choice([-1, 1]) * self.driver.turn_speed
        self.driver.stop()
        if obstacle_fr and not obstacle_fl:
            rospy.loginfo("Obstacle front-right (dist %.2f)!",
                          self.scanner.get_dist('fr'))
            self.ao_turn_speed = abs(self.ao_turn_speed)
        elif obstacle_fl and not obstacle_fr:
            rospy.loginfo("Obstacle front-left (dist %.2f)!",
                          self.scanner.get_dist('fl'))
            self.ao_turn_speed = -abs(self.ao_turn_speed)
        else:
            rospy.loginfo("Obstacle in front (dist %.2f)!",
                          self.scanner.get_dist('front'))

    # TODO: Behaviour class, inheritance
    def enact_behaviour(self):
        if self.ao_in_progress:
            self.driver.turn(self.ao_turn_speed)
        else:
            self.avoid_obstacles_start()

    def cancel_avoid_obstacles(self):
        if self.ao_in_progress:
            self.driver.stop()
            self.driver.move(self.ao_start_speed)
            self.ao_in_progress = False
            self.ao_turn_speed = None
            self.ao_start_speed = None


# Class that implements random walking behaviour
class RandomWalker:
    # Distance & time thresholds
    front_obstacle_dist = 0.5
    fl_obstacle_dist = 0.25
    fr_obstacle_dist = 0.25
    random_walk_dist = 1.5
    random_turn_min_time = 1
    random_turn_max_time = 10

    # Variables used during a random walk
    rwalk_start = None
    rwalk_in_progress = False
    rturn_in_progress = False
    rturn_end_time = None
    rturn_speed = None

    driver = None  # Used to issue movement commands
    pos = None  # Tracks current position

    def __init__(self, driver, pos):
        self.driver = driver
        self.pos = pos

    # Start a random walk
    def start_rwalk(self):
        self.rwalk_start = self.pos.pos()
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
        if dist(self.rwalk_start, self.pos.pos()) < self.random_walk_dist:
            # Continue moving
            self.driver.turn(0)
            self.driver.move(self.driver.move_speed)
            return True
        else:
            # Moved far enough, start turning
            self.driver.move(0)
            self.rwalk_start = None
            self.rwalk_in_progress = False
            self.rturn_in_progress = True
            # Setup turn parameters
            rturn_start = rospy.Time.now()
            rturn_duration = rospy.Duration(
                    random.uniform(self.random_turn_min_time,
                                   self.random_turn_max_time))
            self.rturn_end_time = rturn_start + rturn_duration
            self.rturn_speed = self.driver.turn_speed * random.choice([-1, 1])
            return False

    # Continue an rwalk in the state of turning
    def continue_rwalk_turn(self):
        if rospy.Time.now() < self.rturn_end_time:
            self.driver.move(0)
            self.driver.turn(self.rturn_speed)
            return True
        else:
            self.driver.turn(0)
            self.rturn_in_progress = False
            self.rturn_end_time = None
            self.rturn_speed = None
            return False

    # Cancel an rwalk
    def cancel_rwalk(self):
        if self.is_rwalking():
            self.driver.stop()
            self.rwalk_in_progress = False
            self.rwalk_start = None
            self.rturn_in_progress = False
            self.rturn_end_time = None
            self.rturn_speed = None

    # Start or continue walking randomly
    def enact_behaviour(self):
        if self.is_rwalking():
            self.continue_rwalk()
        else:
            self.start_rwalk()


class ColorBeaconer:
    end_dist_threshold = 0.75  # How close to stop
    # Do these hsv values need tweaking? (we're accepting any S, V values)
    target_hsv_lower = [35,43,46]
    target_hsv_upper = [77, 255, 255]
    # width, width, x-offset, y-offset of the area of the mask that we are
    # considering
    mask_shape = [480, 170, 0, 100]
    cropped_mask = None  # The mask of the target color

    beaconing_in_progress = False
    turn_speed = 0  # Turn direction ('left' or 'right')

    camera = None  # The camera monitor class
    driver = None  # Used to issue movement commands
    scanner = None  # Used to determine if we are close enough to the beacon

    def __init__(self, driver, camera, scanner):
        self.camera = camera
        self.driver = driver
        self.scanner = scanner
        cv2.namedWindow("keypoints", 1)

    # Do we see the color in the camera view?
    '''    def apply_color_mask(self):
        sslower = np.array(self.target_hsv_lower, dtype = "uint8")
        upper = np.array(self.target_hsv_upper, dtype = "uint8")
        mask = cv2.inRange(self.camera.hsv, sslower, upper)
        # Should we apply the mask to the image or just return the mask?
        return mask
'''
    def wants_to_act(self):
        self.update_mask()
        # Return True if the image contains white pixels
        return cv2.countNonZero(self.cropped_mask) != 0

    def update_mask(self):
        mask = self.camera.apply_color_mask(self.target_hsv_lower,
                                            self.target_hsv_upper)
        #mask = self.apply_color_mask()
        # Crop the mask
        w = self.mask_shape[0]
        h = self.mask_shape[1]
        x = self.mask_shape[2]
        y = self.mask_shape[3]
        self.cropped_mask = mask[y:y+h, x:x+w]
        # cv2.imshow("cropped_mask", self.cropped_mask)
        # cv2.waitKey(3)

    # Apply blob detection on the cropped mask image to find the largest blob
    def find_blob(self):
        mask_inv = (255 - self.cropped_mask)
        thresh = cv2.adaptiveThreshold(mask_inv, 255,
                                       cv2.ADAPTIVE_THRESH_MEAN_C,
                                       cv2.THRESH_BINARY, 101, 3)
        # Apply morphology open then close
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        blob = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, kernel)
        # Invert blob
        blob = (255 - blob)
        return blob

    def check_alignment_from_blob(self, blob):
        # Get contours
        cnts = cv2.findContours(blob, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        big_contour = max(cnts, key=cv2.contourArea)  # The largest contour
        x = 0
        ptCnt = len(big_contour)  # Count of points in the contour
        for pt in big_contour:
            x += pt[0][0]
        # The average x position of the contour
        contour_center_x = np.uint32(np.ceil(x/ptCnt))
        kp_im = self.cropped_mask.copy()
        cv2.line(kp_im, (contour_center_x, 0), (contour_center_x, self.mask_shape[1]), (255), thickness=1)
        cv2.imshow("keypoints", kp_im)
        cv2.waitKey(3)

        error = contour_center_x - self.cropped_mask.shape[1]/2
        self.turn_dir = (-float(error) / 400)
                
    def continue_beaconing(self):
        # Place the largest white blob from the cropped_mask in the
        # center of the screen, then start moving forwards.

        # TODO: calculate the approximate location of the beacon object using
        # the current odometry and front laser scan when we are pointing
        # directly at the white blob. Store this location so that we can
        # navigate to it.

        self.update_mask()
        self.check_alignment_from_blob(self.find_blob())

        if self.scanner.get_dist('front') < self.end_dist_threshold:
            rospy.loginfo("Reached beacon!")
            self.driver.stop()
        else:
            self.driver.move(self.driver.move_speed)
            self.driver.turn(self.turn_dir)

    # Do the beaconing behaviour
    def act(self):
        if self.beaconing_in_progress:
            self.continue_beaconing()
        else:
            rospy.loginfo("Beaconing started: moving towards beacon!")
            self.beaconing_in_progress = True

    def cancel_behaviour(self):
        self.driver.stop()
        self.beaconing_in_progress = False



# A class for controlling the behaviours of the robot
class TurtlebotController:
    state = ' '
    state_previous = '_'

    # State of the robot
    driver = None  # Driver
    scanner = None  # LaserScanner
    pos = None  # PositionTracker
    video = None  # Image data

    # Behaviours
    ao = None  # Obstacle Avoider
    rw = None  # Random Walker
    beaconer = None  # Beaconer (for beaconing towards green objects)

    def __init__(self):
        self.driver = Driver()
        self.scanner = LaserScanner()
        self.pos = PositionTracker()
        self.video = Camera()

        self.ao = ObstacleAvoider(self.driver, self.scanner)
        self.rw = RandomWalker(self.driver, self.pos)
        self.beaconer = ColorBeaconer(self.driver, self.video, self.scanner)

        rospy.init_node('driver')
        self.driver.start_timer()
        self.rate = rospy.Rate(10)

        # Wait until we have data from the laser scanner
        while not self.scanner.data_is_valid:
            self.rate.sleep()

    def decide_behaviour(self):
        self.state_previous = self.state
        if self.beaconer.wants_to_act():
            self.state = 'beaconing'  # Beacon towards colored object
            if self.state != self.state_previous:
                rospy.loginfo("See beacon-colored object!"
                              " beaconing towards...")
        elif self.ao.sees_obstacles():
            self.state = 'avoiding'  # Avoiding obstacles
            if self.state != self.state_previous:
                rospy.loginfo("See obstacles! Avoiding them...")
        else:
            self.state = 'randomwalking'  # Randomly walking
            if self.state != self.state_previous:
                rospy.loginfo("Don't see anything! Walking randomly...")

    def enact_behaviour(self):
        if self.state != self.state_previous:
            if self.state_previous == 'randomwalking':
                self.rw.cancel_rwalk()
            elif self.state_previous == 'avoiding':
                self.ao.cancel_avoid_obstacles()
            elif self.state_previous == 'beaconing':
                self.beaconer.cancel_behaviour()
             
        if self.state == 'avoiding':
            self.ao.enact_behaviour()
        elif self.state == 'beaconing':
            self.beaconer.act()
        else:
            self.rw.enact_behaviour()

    def behaviour_loop(self):
        while True:
            self.decide_behaviour()
            self.enact_behaviour()
            self.rate.sleep()


if __name__ == '__main__':
    controller = TurtlebotController()
    try:
        controller.behaviour_loop()
    except KeyboardInterrupt:
        controller.driver.stop()
