#!/usr/bin/env python

import rospy, tf

import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

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
    
    # Ranges
    distances = []

    # The subscriber object
    scan = None
    max_range=3.5
    # Is the scan data valid?
    data_is_valid = False

    def __init__(self):
        self.scan = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        self.distances = msg.ranges
        self.data_is_valid = True
    # Get the distance of a specific angle
    def get_dist(self, angle):
        return self.distances[angle]

    def max_dix(self):
        for i in self.distances:
            if i<self.max_range:
                return True
        return False



# TODO: May have to split into two nodes. One that handles navigation and one that updates the OccupancyGrid. This is because the navigation loop hangs while waiting for a waypoint
# To be reached meaning that updated odometry and laser scan information cannot be fed into the OccupancyGrid. 

class OccupancyGrid():

    # Stores various details about the current grid
    origin = None
    resolution = None

    # Actual storage of the occupancy grid
    grid = []

    # Thresholds for occupied and free grid spaces
    occupiedThresh = 0.5
    freeThresh = 0.25

    def __init__(self, worldOrigin, gridSize, gridResolution):

        self.origin = worldOrigin
        self.resolution = gridResolution

        self.grid = [-1] * gridSize[0] * gridSize[1]

    # May convert incorrectly, converts from a world to a grid coordinate
    def toGrid(self, px, py, size):

        # x = (abs(self.origin[0]) + px) / self.resolution
        # y = (abs(self.origin[1]) + py) / self.resolution
        x = (abs(px - self.origin[0])) / self.resolution
        y = (abs(py - self.origin[1])) / self.resolution

        if x >= size[0] or y >= size[1]:
            return None 

        return (x, y)
        
    # May convert incorrectly, converts from a grid to a world coordinate
    def toWorld(self, gx, gy, size):

        if gx >= size[0] or gy >= size[1]:
            return None

        # 0.5 is added as converting to a continuous data type
        x = self.origin[0] + (gx * self.resolution) + 0.5
        y = self.origin[0] + (gy * self.resolution) + 0.5

        return (x, y)

    # Converts a 2d coordinate to a 1d coordinate for array access
    def toIndex (self, gx, gy, size_x):

        return gy * size_x + gx


class Runner: 

    # Size of our occupancy grid
    gridSize = (384, 384)
    
    pri_x = 0
    pri_y = 0
    pos = None
    scanner = None
    range_max = 3.5

    oGrid = None

    def __init__(self):

        # Creates Odometry and laser scanner objects
        self.pos = PositionTracker()
        self.scanner = LaserScanner()

        # Creates the occupancy grid object and assigns to the variable
        self.oGrid = OccupancyGrid((-10, -10), self.gridSize, 0.05)

        # Initialises the node
        rospy.init_node('occupancyGrid', anonymous = False)

        self.rate = rospy.Rate(10)

    def logInfo(self):

        while True: 
            rospy.loginfo(self.pos.pos())

            self.rate.sleep()
    
    def cal_pos(self,gx,gy,angel,dis):
        x=gx+math.cos(angel)*dis
        y=math.sin(angel)*dis+gy
        return (x,y)


##Determine whether to start drawing grid
    def show_grid(self):
        for i in range(int(math.floor(self.gridSize[0]/10))):
            for j in range(int(math.floor(self.gridSize[0]/10))):
                if self.oGrid.grid[i*10*384+j*10]==0.1:
                    print(' '),
                else:
                    print(self.oGrid.grid[i*10*384+j*10]),
            print('')
   
    def loop(self):
        while True:
            if self.scanner.max_dix():
                self.distan=self.scanner.distances
                self.w_x=self.pos.position['x']
                self.w_y=self.pos.position['y']
                self.w_yaw=self.pos.position['yaw']
                for i in range(len(self.distan)):
                    if self.distan[i]<self.scanner.max_range:
                        print("im drawing")
                        self.draw_grid(i)
                self.show_grid()

    def whether_draw(self,i):
        if self.scanner.get_dist(i)<self.range_max:
            return TRUE

    def draw_grid(self,i):
            dis=self.distan[i]
            angel=(math.pi*i/180+self.w_yaw)%(2*math.pi)
            o_x,o_y=self.cal_pos(self.w_x,self.w_y,angel,dis)
            gx,gy=self.oGrid.toGrid(self.w_x,self.w_y,self.gridSize)
            og_x,og_y=self.oGrid.toGrid(o_x,o_y,self.gridSize)

            g_list= self.get_line((gx,gy),(og_x,og_y))
            
            for i in g_list:
                p_x,p_y=i
                g_index=self.oGrid.toIndex(p_x,p_y,self.gridSize[0])
                self.oGrid.grid[g_index]=0.1
            self.oGrid.grid[g_index]=1
            
    #Bresenham's Line Algorithm
    def get_line(self,start, end):
    # Setup initial conditions
        x1, y1 = int(math.floor(start[0])), int(math.floor(start[1]))
        x2, y2 = int(math.floor(end[0])), int(math.floor(end[1]))
        dx = x2 - x1
        dy = y2 - y1

    # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
 
    # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
 
    # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
 
    # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
 
    # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points


if __name__ == '__main__':
    run = Runner()
    run.loop()
    try:
        run.logInfo()
    except KeyboardInterrupt:
        rospy.loginfo("Exited Node.")
