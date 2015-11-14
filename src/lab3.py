#!/usr/bin/python
import rospy, tf, time, math, roslib
from kobuki_msgs.msg import BumperEvent

from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from std_msgs.msg import Empty, Header

from tf.transformations import euler_from_quaternion

DEBUG = 0
CELL_WIDTH = 0.3
CELL_HEIGHT = 0.3

expanded_cells = []; frontier_cells = []; unexplored_cells = []

#Odometry Callback function.
def odomHandler(msg):
    DBPrint("odomHandler")
    global x, y, theta

    # Update the x, y, and theta globals every time something changes
    quat = msg.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = yaw 

def goalHandler(msg):
    DBPrint('goalHandler')
    global goal_x, goal_y, goal_theta 
    pose = msg.pose

    # Set the goal_x, goal_y, and goal_theta variables
    quat = pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    goal_x = pose.position.x
    goal_y = pose.position.y
    goal_theta = yaw

def mapHandler(msg):
    DBPrint('mapHandler')
    global expanded_cells, frontier_cells, unexplored_cells

    map_width = msg.info.width
    map_height = msg.info.height
    occupancy = msg.data

    index = 0

    for row in range(1, map_height):
        for col in range(1, map_width):
            if occupancy[index] == -1:
                publishCell(row, col, 'unexplored')
            elif occupancy[index] == 100:
                publishCell(row, col, 'frontier')
            else:
                publishCell(row, col, 'expanded')


    '''
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    nav_msgs/MapMetaData info
        time map_load_time
        float32 resolution
        uint32 width
        uint32 height
        geometry_msgs/Pose origin
            geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
            geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
    int8[] data
    ''' 
# Creates and adds a cell to its corresponding list to be published in the near future
def publishCell(x, y, state):
    DBPrint('publishCell')
    global expanded_cells, frontier_cells, unexplored_cells

    if state == 'expanded':
        expanded_cells.append(GridCell(x, y))
    elif state == 'frontier':
        frontier_cells.append(GridCell(x, y))
    elif state == 'unexplored':
        unexplored_cells.append(GridCell(x, y))
    else:
        print 'Bad state'

# Publishes a message to display all of the cells
def publishCells():
    DBPrint('publishCells')
    publishExpanded()
    publishFrontier()
    publishUnexplored()

# Publishes the information stored in expanded_cells to the map
def publishExpanded():
    DBPrint('publishExpanded')
    pub_expanded = rospy.Publisher('/expanded_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    points = []

    for cell in expanded_cells:
        point = Point()
        point.x = cell.x
        point.y = cell.y
        point.z = 0
        points.append(point)

    msg.cells = points
    pub_expanded.publish(msg)

# Publishes the information stored in frontier_cells to the map
def publishFrontier():
    DBPrint('publishFrontier')
    pub_frontier = rospy.Publisher('/frontier_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT
    
    points = []

    for cell in frontier_cells:
        point = Point()
        point.x = cell.x
        point.y = cell.y
        point.z = 0
        points.append(point)

    msg.cells = points
    pub_frontier.publish(msg)

# Publishes the information stored in unexplored_cells to the map
def publishUnexplored():
    DBPrint('publishUnexplored')
    pub_unexplored = rospy.Publisher('/unexplored_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    points = []

    for cell in unexplored_cells:
        point = Point()
        point.x = cell.x
        point.y = cell.y
        point.z = 0
        points.append(point)

    msg.cells = points
    pub_unexplored.publish(msg)

def AStar():
    DBPrint('AStar')


# If you need something to happen repeatedly at a fixed interval, write the code here.
# rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    DBPrint("timerCallback")
    publishCells()
    
    
def DBPrint(param):
    if DEBUG == 1:
        print param

def main():
    DBPrint('main')
    global vel_pub

    rospy.init_node('lab3')

    # Publisher for commanding robot motion
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    
    # Subscribe to bumper changes
    # rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperHandler, queue_size=1) 
    
    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odomHandler)

    # Subscribe to the map
    rospy.Subscriber('/map', OccupancyGrid, mapHandler)

    # Subscribe to NavToGoal stuff
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalHandler)

    # Create Odemetry listener and boadcaster 
    odom_list = tf.TransformListener()
    # odom_tf = tf.TransformBroadcaster()
   
    # Wait for an odom event to init pose
    rospy.sleep(rospy.Duration(1))

    # Update the map cells every second
    rospy.Timer(rospy.Duration(1), timerCallback)
    
    rospy.spin()


if __name__ == '__main__':
    main()
