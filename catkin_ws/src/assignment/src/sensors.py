import rospy, tf, cv2, cv_bridge, actionlib, numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point



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
    # The named distances to track
    dists = {'front': {'dist': None, 'fov': 10, 'angle': 0},
             'f1' : {'dist': None, 'fov': 1, 'angle': 0},
             'fl': {'dist': None, 'fov': 5, 'angle': 45},
             'left': {'dist': None, 'fov': 0, 'angle': 90},
             'back': {'dist': None, 'fov': 0, 'angle': 180},
             'right': {'dist': None, 'fov': 0, 'angle': 270},
             'fr': {'dist': None, 'fov': 5, 'angle': 315}}
    # Ranges
    distances = []
    dist_range = (-1, -1)

    # The subscriber object
    scan = None

    # Is the scan data valid?
    data_is_valid = False

    # Function to update the occupancy grid (passed as an argument when
    # creating this object, contains a reference to the oGrid instance baked
    # in, so we don't have to reference the grid directly).
    # def update_oGrid(distances, dist_range): ...
    update_oGrid = None
    mark_oGrid_updated = None

    def __init__(self, update_oGrid=None, mark_oGrid_updated=None):
        self.scan = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.update_oGrid = update_oGrid
        self.mark_oGrid_updated = mark_oGrid_updated

    def scan_callback(self, msg):
        self.dist_range = (msg.range_min, msg.range_max)
        self.distances = msg.ranges
        for d in self.dists.keys():
            self.set_named_dist(d, msg.ranges)
        if self.update_oGrid is not None:
            # TODO: store the position at the current time (or how else to
            # prevent an updated position being used to update the oGrid with
            # outdated laser scan data?)
            rospy.Timer(0.05,
                        lambda _: self.update_oGrid(self.distances, self.dist_range),
                        oneshot=True)
            # for a, d in enumerate(self.distances):
            #     self.update_oGrid(d, self.dist_range, a)
        if self.mark_oGrid_updated is not None:
            self.mark_oGrid_updated()
        self.data_is_valid = True

    # Get the distance of a specific angle
    def get_dist(self, angle):
        return self.distances[angle]

    def set_named_dist(self, dir_name, ranges):
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
    def get_named_dist(self, dir_name):
        if ((self.data_is_valid) and (dir_name in self.dists)):
            return self.dists[dir_name]['dist']
        else: 
            return None

# Class that handles rbg camera feed (subscribes to /camera/rgb/image_raw)
class Camera:
    hsv = None  # The HSV image
    bridge = None  # The opencv bridge
    video = None  # The video subscriber

    data_is_valid = False

    def __init__(self):
        self.video = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()

        cv2.namedWindow("Camera Video", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        image_resized = cv2.resize(image, (w/4,h/4))
        self.hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

        cv2.imshow("Camera Video", image_resized)

        self.data_is_valid = True

    def apply_color_mask(self, lower, upper):
        if self.data_is_valid:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(self.hsv, lower, upper)
            # Should we apply the mask to the image or just return the mask?
            return mask
        else: 
            return None

    def getImage(self):
        return self.hsv

# Class that handles depth camera feed (subscribes to /camera/depth/image_raw)
class DepthCamera:

    # Stores the original and normalised depth images
    depthImage = None
    normalisedDepthImage = None

    # Video feed as well as the cv2 bride used to convert ros messages to something cv2 can understand
    video = None
    bridge = None

    # Used to check if data is valid
    data_is_valid = False

    # Max range of depth camera is 5m
    max_depth = 5 

    # Initialises the video and bridge variables
    def __init__(self):
        self.video = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.bridge = cv_bridge.CvBridge()


    def depth_callback(self, msg): 
        # Sets the original depth image by passing it through the bridge. 
        # Each pixel is the range to the 
        self.depthImage = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Normalises the pixel values to be between 0 and 1
        image_norm = cv2.normalize(self.depthImage , self.depthImage , 0, 1, cv2.NORM_MINMAX)

        # Resizes the image so it can be displayed easier
        (h, w) = image_norm .shape[:2]
        self.normalisedDepthImage = cv2.resize(image_norm, (w/4,h/4))

        # Validates the data
        self.data_is_valid = True

    # Returns the normalised depth image
    def getNormalisedDepthImage(self):
        return self.normalisedDepthImage

    # Returns the raw depth image
    def getDepthImage(self):
        return self.depthImage

class navigator:

    # Dictionary of waypoints and the coordinated
    waypoints = {
        0 : {'x' : -2.85, 'y' : -3.1},
        1 : {'x' : -1.95, 'y' : -1.95},
        2 : {'x' : -0.95, 'y' : -4.75},
        3 : {'x' : 0.8, 'y' : -3.3},
        4 : {'x' : 2.35, 'y' : -0.6},
    }

    # Variables storing if goal has been reached, and the OccupancyGrid object
    goalReached = False
    oGrid = None

    # def __init__(self):

        # Initialises the node
        

    def navigate_to_point(self,index_num):
     
        # What happens when navigation cannot reach destination, should we stop completly or carry on to next waypoint?

        # Loops through each waypoint in the dictionary and calls moveToGoal() on each set of coordinates.
        rospy.loginfo("Goal: Waypoint %i", index_num)
        self.goalReached = self.moveToGoal(self.waypoints[index_num]['x'], self.waypoints[index_num]['y'])
            # If goal successfully reached logs so, and if unnsuccessful then also logs
        if self.goalReached:
            rospy.loginfo("Reached waypoint %i!", index_num)
        else:
            rospy.loginfo("Waypoint %i not reached successfully. :(", i+1)

        # Logs finished navigation
        rospy.loginfo("Finished navigation")

    def moveToGoal(self, xGoal, yGoal):

        # Create action client which communicates with the action client server over a number of topics to send messages to the robot
        actionClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the client to connect to the server so that commands can be sent to the robot
        while(not actionClient.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for action client server...")

        # Create MoveBaseGoal message
        goal = MoveBaseGoal()

        # Sets the frame_id which sets how the coordinates will be considered. In this case it uses the 'map' frame id which uses absolute coordinates 
        goal.target_pose.header.frame_id = "map"
        # Adds a timestamp to the message
        goal.target_pose.header.stamp = rospy.Time.now()

        # Sets the goal for coordinate and heading
        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 2.0

        # Send the goal to the server
        actionClient.send_goal(goal)

        rospy.loginfo("Moving towards goal...")

        # Wait for the result to be returned. This is used to see if the goal has been successfully reached
        actionClient.wait_for_result()
        if (actionClient.get_result() != None):
            return True
        else:
            return False

