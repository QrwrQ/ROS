#!/usr/bin/env python

from numpy.lib.function_base import angle
from numpy.lib.twodim_base import mask_indices
import rospy, random, cv2, math, numpy as np, sys
import rospkg
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from lib import util
from lib.sensors import PositionTracker, LaserScanner, Camera, DepthCamera

# Class that drives the robot (publishes to topic /cmd_vel)
class Driver:
    move_speed = 0.2
    turn_speed = 0.2

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

# Class that implements obstacle avoidance behaviour
class ObstacleAvoider:
    # Constants for modifying behaviour
    # How much further to scan when we've seen an obstacle
    alert_range_modifier = 2

    distance_threshold = 0.1

    # Obstacle distance thresholds for each direction (how close before we
    # start avoiding)
    obstacle_dists = {'f1': 0.5,
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
        d = self.scanner.get_named_dist(dir_name)
        return d < dist_threshold

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
                          self.scanner.get_named_dist('fr'))
            self.ao_turn_speed = abs(self.ao_turn_speed)
        elif obstacle_fl and not obstacle_fr:
            rospy.loginfo("Obstacle front-left (dist %.2f)!",
                          self.scanner.get_named_dist('fl'))
            self.ao_turn_speed = -abs(self.ao_turn_speed)
        else:
            rospy.loginfo("Obstacle in front (dist %.2f)!",
                          self.scanner.get_named_dist('f1'))

    # Convert a distance from the center of the robot into a distance from the
    # outside of the robot. Treats the robot as a square
    def dist_to_robot_edge(self, a, d):
        base = 0.13  # Half the diameter of the robot shape
        period = 90  # The period of the triangle wave
        amplitude = math.sqrt(2)*base  # The max height of the wave
        y = 2*amplitude * abs((a/period) - math.floor((a/period) + (1/2)))
        return d - (y + base)

    # Returns the direction that is closest to an obstacle if any direction is
    # less than a threshold
    def closest_scanline(self):
        closest = (3.5, -1)  # (distance, angle)
        for a in range(0, 360):
            d = self.scanner.get_dist(a)
            d_o = self.dist_to_robot_edge(a, d)
            # if d_o < 0 or d_o > 100 and d_o != float('inf'):
            #     print('angle %d distance %.2f (%.2f)' % (a, d_o, d))
            if d_o < closest[0]:
                closest = (d_o, a)
        return closest

    def enact_behaviour(self):
        if self.ao_in_progress:
            # Can't turn when too close to a wall, need to move either
            # backwards or forwards
            (closest_distance, closest_angle) = self.closest_scanline()
            if closest_distance < self.distance_threshold: 
                rospy.loginfo("Obstacle too close to safely turn (angle %d - distance %.2f)!"
                        " moving to avoid...", closest_angle, closest_distance)
                # We move forwards or backwards depending on the angle
                if closest_angle > 90 and closest_angle < 270:
                    # Move forwards - closest object is behind us
                    self.driver.move(self.driver.move_speed)
                else:
                    # Move backwards - closest object is in front
                    self.driver.move(-self.driver.move_speed)
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
        if util.dist(self.rwalk_start, self.pos.pos()) < self.random_walk_dist:
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

# TODO: count the number of visited beacons of each type.
class BeaconMapper:
    # Various details about the map
    origin = None
    resolution = None
    size = None

    # The map of visited beacons
    beacon_map = None
    env_map = None

    def __init__(self, w_origin, (w, h), resolution):
        self.origin = w_origin
        self.resolution = resolution
        self.size = (w, h)
        self.beacon_map = [0] * (w*h)
        # TODO: publish the map so we can subscribe to it instead of reading the image
        # FIXME: STOP USING ABSOLUTE PATHS! Everyone else has to change the
        # path any time you push your absolute path
        # - Use __file__ variable to get the absolute path of the file and go
        #   from there?
        # - If you MUST use an absolute path, avoid including it in your
        #   commits! Learn how to stage/unstage hunks and ensure that the hunk
        #   containing your absolute path isn't staged.
        rospack = rospkg.RosPack()
        self.env_map = cv2.flip(cv2.imread(rospack.get_path('comp4034')+"/src/assignment/maps/train_env.pgm"), 0)
        

        cv2.namedWindow("Beacon Map", cv2.WINDOW_NORMAL)


    # The size of the beacon depending on the beacon type - these are estimated
    # and should probably be made larger but not too much larger
    def beacon_size(self, beacon_num):
        if beacon_num == 1:
            # Green box, size 0.5x0.5
            return 0.7
        elif beacon_num == 2:
            # Red fire hydrant, size 0.4x0.4
            return 0.6
        elif beacon_num == 3:
            # Blue postbox, size 0.8x0.8
            return 1.0

    def world_to_map(self, (wx, wy)):
        x = (wx - self.origin[0]) / self.resolution
        y = (wy - self.origin[1]) / self.resolution
        if x >= self.size[0] or y >= self.size[1]:
            return None 
        return (x, y)

    def visit_beacon(self, bot_pos, bot_yaw, beacon_depth, beacon_type):
        # Calculate the position of the beacon (using the depth)
        beacon_size = self.beacon_size(beacon_type)
        # Go beacon_size/2 further inside the beacon so that we get
        # approximately the center of the beacon
        pos = util.line_end(bot_pos, bot_yaw, beacon_depth+beacon_size/2)
        m_pos = self.world_to_map(pos)
        if m_pos is None:
            return
        (m_x, m_y) = m_pos
        # The beacon position as an index into the map
        idx = m_x + m_y * self.size[0]
        # Mark an area of the internal beacon_map as visited
        # The size of the beacon scaled to the beacon map
        beacon_map_size = beacon_size/self.resolution
        self.mark_square_visited((m_x, m_y), 1.5*beacon_map_size, beacon_type)

    #mark the color information in the beacon map
    def mark_square_visited(self, (c_x, c_y), beacon_size, beacon_num):
        # Calculate the corners of the beacon
        (tl_x, tl_y) = (c_x - beacon_size/2, c_y - beacon_size/2)
        (br_x, br_y) = (c_x + beacon_size/2, c_y + beacon_size/2)
        w = self.size[0]
        # Iterate through the rows the beacon occupies, marking them with the
        # beacon_num on the map
        try:
            for y in range(int(tl_y), int(br_y+1)):
                if y < 0 or y >= self.size[1]:
                    continue
                for x in range(int(tl_x), int(br_x+1)):
                    if x < 0 or x >= w:
                        continue
                    idx = x + y*w
                    self.beacon_map[idx] = beacon_num
        except ValueError:
            return

    def update_map_image(self, (wx, wy), yaw):
        line_length = math.sqrt(self.size[0]**2+self.size[1]**2)
        m_pos = self.world_to_map((wx, wy))
        if m_pos is None:
            print("Not updating map image")
            return
        # Get the end-point of the line
        line_end = util.line_end(m_pos, yaw, line_length)
        # Default image
        # img = np.zeros([self.size[0], self.size[1], 3])
        img = self.env_map.copy()
        # Fill in the beacons that have been visited
        for (idx, pt) in enumerate(self.beacon_map):
            x = idx % self.size[0]
            y = (idx - x) / self.size[0]
            if pt != 0:
                if pt == 1:  # Fire hydrant, red
                    img[y][x] = (0, 255, 0)
                elif pt == 2:  # Green box, green
                    img[y][x] = (0, 0, 255)
                elif pt == 3:  # Postbox, blue
                    img[y][x] = (255, 0, 0)
        # Draw the view line
        cv2.line(img, (int(m_pos[0]), int(m_pos[1])), (int(line_end[0]),
                 int(line_end[1])), (0, 0, 0))
        img = cv2.flip(img, 1)
        cv2.imshow('Beacon Map', img)

    # Check if there are any of the given type of beacon in the given direction
    # Returns the distance to the beacon, if there was one
    # By default checks for any kind of beacon (beacon_type=0)
    def beacons_in_direction(self, (wx, wy), yaw, beacon_type=0):
        self.update_map_image((wx, wy), yaw)
        # Maximum distance is the length of the diagonal of the map
        line_length = math.sqrt(self.size[0]**2+self.size[1]**2)
        # Get the map position to start the search from
        m_pos = self.world_to_map((wx, wy))
        if m_pos is None:
            return 0
        # Get the end-point of the line
        line_end = util.line_end(m_pos, yaw, line_length)
        # Get the points on the line
        line = util.line_points(m_pos, line_end)
        # Iterate through the line points and check for beacons
        w = self.size[0]
        for (x, y) in line:
            if x < 0 or x >= self.size[0]:
                continue
            if y < 0 or y >= self.size[1]:
                continue
            idx = x + y*w
            map_obj = self.beacon_map[idx]
            distance = math.sqrt((x-wx)**2 + (y-wy)**2)
            if map_obj != 0:
                if beacon_type == 0 or beacon_type == map_obj:
                    return distance
        return -1  # No beacons found



class ColorBeaconer:
    end_dist_threshold = 0.75  # How close to stop

    target_hsv_lower_green = [52, 255,   0]
    target_hsv_upper_green = [80, 255, 255]

    target_hsv_lower_red = [0, 240, 0] #0,43,46
    target_hsv_upper_red = [40, 255, 255]

    target_hsv_lower_blue = [83, 96, 0]
    target_hsv_upper_blue = [114,255,150]
    # width, height, x-offset, y-offset of the area of the mask that we are
    # considering
    mask_shape = [480, 270, 0, 0]
    cropped_mask = None  # The mask of the beacons
    merged_mask = None  # A 3 channel image storing 3 colors' masks

    color_num=0

    beaconing_in_progress = False
    turn_speed = 0  # Turn direction ('left' or 'right')

    top5_contours = []  # Used to ensure we continue tracking the same blob
    top5_blue = []
    top5_green = []
    top5_red = []

    camera = None  # The camera monitor class
    # TODO: Use the depth camera to determine the position of the beacon when
    # it is within range. (What to do when beacon is outside range? Assume 5m?)
    depthcam = None
    depthcam_max_depth = 5
    driver = None  # Used to issue movement commands
    scanner = None  # Used to determine if we are close enough to the beacon

    # last_visit_time = 0
    tgt_center_x = None
    tgt_center_x = None

    # Function to approximate the position of whatever we are looking at
    # Requires no arguments, but can optionally accept the name of a direction
    get_target_pos = None
    aligned = False  # We update the target pos only if we are pointing at it
    target_pos = None
    # TODO: use navigation to get to the beacon once we have determined its
    # position
    # - Use depth camera to get beacon position more accurately and from a
    # larger distance?

    def __init__(self, driver, camera, scanner, depthcam, get_target_pos=None):
        self.last_visit_time = rospy.Time(0)
        self.camera = camera
        self.driver = driver
        self.scanner = scanner
        self.depthcam = depthcam
        self.get_target_pos = get_target_pos

        cv2.namedWindow("Beacon mask", cv2.WINDOW_NORMAL)

    # Do we see the color in the camera view?
    def wants_to_act(self):
        self.update_mask()
        b, g, r = cv2.split(self.merged_mask)

        # Return True if the image contains more than 800 nonzero (b, g, or r)
        # pixels
        pix_count = cv2.countNonZero(b) + cv2.countNonZero(g) + cv2.countNonZero(r)
        return pix_count >= 800

    def update_mask_image(self):
        self.update_mask()
        mask_im = self.merged_mask.copy()
        if self.tgt_center_x is not None:
            cv2.line(mask_im, (self.tgt_center_x, 0), (self.tgt_center_x,
                                                    self.mask_shape[1]),
                     (255, 255, 255), thickness=1)
        cv2.imshow('Beacon mask', mask_im)

    def update_mask(self):
        mask_green = self.camera.apply_color_mask(self.target_hsv_lower_green,
                                            self.target_hsv_upper_green)
        mask_red=self.camera.apply_color_mask(self.target_hsv_lower_red,
                                            self.target_hsv_upper_red)
        mask_blue=self.camera.apply_color_mask(self.target_hsv_lower_blue,
                                            self.target_hsv_upper_blue)
        # Crop the mask
        w = self.mask_shape[0]
        h = self.mask_shape[1]
        x = self.mask_shape[2]
        y = self.mask_shape[3]
        cropped_mask_green = mask_green[y:y+h, x:x+w]
        cropped_mask_red = mask_red[y:y+h, x:x+w]
        cropped_mask_blue = mask_blue[y:y+h, x:x+w]
        
        # Removes small islands in the masks
        opening_filter = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        closing_filter = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        opened_red_mask = cv2.morphologyEx(cropped_mask_red, cv2.MORPH_OPEN, opening_filter)
        opened_green_mask = cv2.morphologyEx(cropped_mask_green, cv2.MORPH_OPEN, opening_filter)
        opened_blue_mask = cv2.morphologyEx(cropped_mask_blue, cv2.MORPH_OPEN, opening_filter)
        
        closed_red_mask = cv2.morphologyEx(opened_red_mask, cv2.MORPH_OPEN, closing_filter)
        closed_green_mask = cv2.morphologyEx(opened_green_mask, cv2.MORPH_OPEN, closing_filter)
        closed_blue_mask = cv2.morphologyEx(opened_blue_mask, cv2.MORPH_OPEN, closing_filter)

        # Merges the masks
        self.merged_mask = cv2.merge([closed_blue_mask, closed_green_mask, closed_red_mask])
        
        
    # Apply blob detection on the cropped mask image to find the largest blob
    def find_blob(self, single_channel_image):
        mask_inv = (255 - single_channel_image)
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

    def get_contours_from_blob(self, blob, previous_top5):
        cnts = cv2.findContours(blob, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = filter(lambda c: cv2.contourArea(c) > 500, cnts)
        cnts.sort(reverse=True, key=cv2.contourArea)
        cnts = cnts[0:5]  # Look at the largest 5 objects in our current view
        # Temporary list of top5 largest contours (and the time we first saw
        # them)
        new_top5 = []
        for cnt in cnts:
            cnt_size = cv2.contourArea(cnt)
            # Get the center of the object
            x = 0
            ptCnt = len(cnt)
            for pt in cnt:
                x += pt[0][0]  # Sum the x positions of the blob
            # Average x position (rounded to an integer)
            cnt_avg_x = np.int16(x/ptCnt)

            # Search for this object in the list of 5 we are tracking
            # if we find it, update the location in a new list
            # if not, add this object to the new list
            found = False
            for i in range(len(previous_top5)):
                (b_x, t, _) = previous_top5[i]
                # If this is an object we were already tracking, update its
                # position
                if abs(b_x - cnt_avg_x) < 10:  # FIXME: put threshold in class constant
                    new_top5.append((cnt_avg_x, t, cnt_size))
                    found = True
            if not found:
                new_top5.append((cnt_avg_x, rospy.Time.now(), cnt_size))
        new_top5.sort(key=lambda (x, t, sz): t)
        return new_top5[0:5]

    # Calculate the angle between the robot and the beacon
    def get_angle(self):
        if self.cropped_mask is None:
            return 0
        if self.tgt_center_x is not None:
            x_dis = self.mask_shape[0]/2-self.tgt_center_x
            a = math.atan(x_dis/self.mask_shape[1])
            return a
        return 0
                
    def continue_beaconing(self):
        # Place the largest white blob from the cropped_mask in the
        # center of the screen, then start moving forwards.

        self.update_mask()
        # The masks
        b, g, r = cv2.split(self.merged_mask)
        # The blobs
        blb_b = self.find_blob(b)
        blb_g = self.find_blob(g)
        blb_r = self.find_blob(r)
        # Update the largest 5 contours for each color
        self.top5_blue = self.get_contours_from_blob(blb_b, self.top5_blue)
        self.top5_green = self.get_contours_from_blob(blb_g, self.top5_green)
        self.top5_red = self.get_contours_from_blob(blb_r, self.top5_red)
        # Determine if there are any contours
        no_b = len(self.top5_blue) == 0
        no_g = len(self.top5_green) == 0
        no_r = len(self.top5_red) == 0
        if no_b and no_g and no_r:
            return 0
        # Get the largest contour for each color (and their size)
        cnt_b = None if no_b else self.top5_blue[0]
        cnt_g = None if no_g else self.top5_green[0]
        cnt_r = None if no_r else self.top5_red[0]
        # Put the sizes into a dict
        cnts_sizes = {'b': 0,
                      'g': 0,
                      'r': 0}
        if not no_b:
            cnts_sizes['b'] = cnt_b[2]
        if not no_g:
            cnts_sizes['g'] = cnt_g[2]
        if not no_r:
            cnts_sizes['r'] = cnt_r[2]
        # Check which contour is largest
        if cnts_sizes['b'] > cnts_sizes['g'] and cnts_sizes['b'] > cnts_sizes['r']:
            tgt_contour = cnt_b
            self.color_num = 3
        elif cnts_sizes['g'] > cnts_sizes['b'] and cnts_sizes['g'] > cnts_sizes['r']:
            tgt_contour = cnt_g
            self.color_num = 1
        elif cnts_sizes['r'] > cnts_sizes['b'] and cnts_sizes['r'] > cnts_sizes['g']:
            tgt_contour = cnt_r
            self.color_num = 2
        else:
            print("unreachable!")
            if cnts_sizes['b'] > 0:
                tgt_contour = cnt_b
                self.color_num = 3
            elif cnts_sizes['g'] > 0:
                tgt_contour = cnt_g
                self.color_num = 1
            elif cnts_sizes['r'] > 0:
                tgt_contour = cnt_r
                self.color_num = 2
            else:
                print("No contours! unreachable")
                return

        # Calculate the error and turn direction
        self.tgt_center_x = tgt_contour[0]
        error = self.tgt_center_x - self.merged_mask.shape[1]/2
        if abs(error) < 5:  # TODO: put threshold in class constant
            self.aligned = True
        else:
            self.aligned = False
        self.turn_dir = (-float(error) / 300)  # Proportional controller

        # Update the stored position of where the target is. This will be used
        # to navigate to the beacon if we lose track of it at some point.
        if self.aligned and self.get_target_pos is not None:
            self.target_pos = self.get_target_pos()

        if self.scanner.get_named_dist('f1') < self.end_dist_threshold:

            if self.color_num == 1:
                rospy.loginfo("Reached Green Beacon!")
            elif self.color_num == 2:
                rospy.loginfo("Reached Red Beacon!")
            elif self.color_num == 3:
                rospy.loginfo("Reached Blue Beacon!")
         
            # self.last_visit_time = rospy.get_rostime()

            self.driver.stop()
            return self.color_num
        elif self.scanner.get_named_dist('f1') > 2*self.end_dist_threshold and self.color_num == 3 and (self.scanner.get_named_dist('fl')**2+self.scanner.get_named_dist('fr')**2) < 1.5:
            rospy.loginfo("Reached Blue Beacon!")
            self.driver.stop()
            return self.color_num

        else:
            self.driver.move(self.driver.move_speed)
            self.driver.turn(self.turn_dir)
            return 0

    # Do the beaconing behaviour
    def act(self):
        if self.beaconing_in_progress:
            return self.continue_beaconing()
        else:
            rospy.loginfo("Beaconing started: moving towards beacon!")
            self.beaconing_in_progress = True

            return 0

    def cancel_behaviour(self):

        self.driver.stop()
        self.merged_mask = None
        self.beaconing_in_progress = False
        self.aligned = False
        self.tgt_center_x = None
        self.turn_dir = ''
        self.top5_contours = []  # Forget the objects we are tracking

# A class for controlling the behaviours of the robot
class TurtlebotController:
    # State of the robot
    state = ' '
    state_previous = '_'

    # Robot Movement + Sensors
    driver = None  # Driver
    scanner = None  # LaserScanner
    pos = None  # PositionTracker
    video = None  # Image data
    depth = None  # Depth camera

    # Behaviours
    ao = None  # Obstacle Avoider
    rw = None  # Random Walker
    beaconer = None  # Beaconer (for beaconing towards green objects)

    def __init__(self):
        self.driver = Driver()
        self.scanner = LaserScanner()
        self.pos = PositionTracker()
        self.video = Camera()
        # Depth camera can see about 5m (which is further than laserscan range)
        self.depth = DepthCamera()

        self.beacon_map = BeaconMapper((-10, -10), (384, 384), 0.05)

        self.ao = ObstacleAvoider(self.driver, self.scanner)
        self.rw = RandomWalker(self.driver, self.pos)
        self.beaconer = ColorBeaconer(self.driver, self.video, self.scanner,
                                      self.depth, self.approx_object_pos)

        rospy.init_node('driver')
        self.driver.start_timer()
        self.rate = rospy.Rate(10)

        # Wait until we have data from the laser scanner
        while not self.scanner.data_is_valid:
            self.rate.sleep()

        while not self.video.data_is_valid:
            self.rate.sleep()

    # The approximate position of the object we are looking at
    def approx_object_pos(self, dir_name='f1'):
        if None in [self.scanner, self.pos]:
            return None
        (x, y) = self.pos.pos()
        a = self.scanner.dists[dir_name]['angle']
        if a == float('inf'):
            a = 3.5  # FIXME: magic number
        d = self.scanner.get_named_dist(dir_name)
        pos = util.line_end((x, y), a, d)
        return pos

    def decide_behaviour(self):
        self.state_previous = self.state
        if self.ao.sees_obstacles():
            self.state = 'avoiding'  # Avoiding obstacles
            if self.state != self.state_previous:
                rospy.loginfo("See obstacles! Avoiding them...")
        elif self.beaconer.wants_to_act() and self.beacon_map.beacons_in_direction(self.pos.pos(), self.pos.yaw()+self.beaconer.get_angle()) == -1:
            self.state = 'beaconing'  # Beacon towards colored object
            if self.state != self.state_previous:
                if self.beaconer.color_num==1:
                    rospy.loginfo("See beacon-colored_green object!"
                                " beaconing towards...")
                if self.beaconer.color_num==2:
                    rospy.loginfo("See beacon-colored_red object!"
                                " beaconing towards...")
                if self.beaconer.color_num==3:
                    rospy.loginfo("See beacon-colored_blue object!"
                                " beaconing towards...")
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
                print("Cancelled Beaconing")
                self.beaconer.cancel_behaviour()
             
        if self.state == 'avoiding':
            self.ao.enact_behaviour()
        elif self.state == 'beaconing':
            beacon_type = self.beaconer.act()
            if beacon_type != 0:  # If we reached the beacon
                pos = self.pos.pos()
                yaw = self.pos.yaw()
                depth = self.scanner.get_named_dist("f1")
                if depth > self.beaconer.end_dist_threshold+0.5:
                    depth = 0.5
                self.beacon_map.visit_beacon(pos, yaw, depth, beacon_type)
        else:
            self.rw.enact_behaviour()

    def behaviour_loop(self):
        while True:
            self.decide_behaviour()
            self.enact_behaviour()
            self.beacon_map.update_map_image(self.pos.pos(), self.pos.yaw())
            self.beaconer.update_mask_image()
            cv2.waitKey(1)
            self.rate.sleep()


if __name__ == '__main__':
    controller = TurtlebotController()
    try:
        controller.behaviour_loop()
    except KeyboardInterrupt:
        controller.driver.stop()
        cv2.destroyAllWindows()
