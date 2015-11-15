#!/usr/bin/python
import rospy, tf, time, math, roslib
from kobuki_msgs.msg import BumperEvent

from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from std_msgs.msg import Empty, Header

from tf.transformations import euler_from_quaternion

from GridCell import GridCell

DEBUG = 0
CELL_WIDTH = 0.3
CELL_HEIGHT = 0.3

expanded_cells = []; frontier_cells = []; unexplored_cells = []

#Odometry Callback function.
def odomHandler(msg):
    DBPrint("odomHandler")
    global x, y, theta, x_cell, y_cell

    try:
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

        # Update the x, y, and theta globals every time something changes
        roll, pitch, yaw = euler_from_quaternion(rot)

        x = trans[0]
        y = trans[1]
        theta = yaw
        x_cell = int((x - 2* CELL_WIDTH) // CELL_WIDTH)
        y_cell = int(y // CELL_WIDTH)
    except:
        print "Waiting for transform..."

def goalHandler(msg):
    DBPrint('goalHandler')
    global goal_x, goal_y, goal_theta, x_goal_cell, y_goal_cell 
    pose = msg.pose

    # Set the goal_x, goal_y, and goal_theta variables
    quat = pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    goal_x = pose.position.x
    goal_y = pose.position.y
    goal_theta = yaw
    
    x_goal_cell = int((goal_x - 2* CELL_WIDTH) // CELL_WIDTH)
    y_goal_cell = int(goal_y // CELL_WIDTH)

    print 'Goal ',  x_goal_cell, y_goal_cell
    AStar()


def mapHandler(msg):
    DBPrint('mapHandler')
    global expanded_cells, frontier_cells, unexplored_cells, CELL_WIDTH, CELL_HEIGHT
    global map_width, map_height, occupancyGrid

    map_width = msg.info.width
    map_height = msg.info.height
    occupancyGrid = msg.data

    CELL_WIDTH = msg.info.resolution
    CELL_HEIGHT = msg.info.resolution

    x_offset = msg.info.origin.position.x + (2*CELL_WIDTH) 
    y_offset = msg.info.origin.position.y - (2*CELL_HEIGHT)

    index = 0

    for row in range(1, map_height+1):
        for col in range(1, map_width+1):
            index = (row - 1) * map_width + (col - 1)
            if occupancyGrid[index] == -1:
                publishCell(col+x_offset, row+y_offset, 'unexplored')
            elif occupancyGrid[index] == 100:
                publishCell(col+x_offset, row+y_offset, 'frontier')
            else:
                publishCell(col+x_offset, row+y_offset, 'expanded')


# Creates and adds a cell-location to its corresponding list to be published in the near future
def publishCell(x, y, state):
    DBPrint('publishCell')
    global expanded_cells, frontier_cells, unexplored_cells

    p = Point()
    p.x = x*CELL_WIDTH
    p.y = y*CELL_HEIGHT
    p.z = 0

    if state == 'expanded':
        expanded_cells.append(p)
    elif state == 'frontier':
        frontier_cells.append(p)
    elif state == 'unexplored':
        unexplored_cells.append(p)
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

    msg.cells = expanded_cells
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

    msg.cells = frontier_cells
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

    msg.cells = unexplored_cells
    pub_unexplored.publish(msg)


# If you need something to happen repeatedly at a fixed interval, write the code here.
# rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    DBPrint("timerCallback")
    publishCells()
    
    
def DBPrint(param):
    if DEBUG == 1:
        print param

def AStar():
    global x_cell, y_cell, x_goal_cell, y_goal_cell

    # Create the costMap
    costMap = [[0 for x in range(map_width)] for x in range(map_height)]

    count = 0
    # iterating through every position in the matrix
    print occupancyGrid
    for row in range(0, map_height):
        for col in range (0, map_width):
            costMap[row][col] = GridCell(row, col, occupancyGrid[count]) # creates all the gridCells
            costMap[row][col].setH(x_goal_cell, y_goal_cell) # adds an H value to every gridCell
            count += 1

    # Keep track of explored cells
    explored = []

    # Keep track of path
    path = []

    # make the start position the selected cell
    selectedCell = costMap[x_cell][y_cell]

    while(selectedCell != costMap[x_goal_cell][y_goal_cell]): 
        path.append(selectedCell) 
        toExplore = unexploredNeighbors(selectedCell, explored, costMap) 
        explored.extend(toExplore)
        tentativeNextCell = toExplore[0]
        for n in toExplore:
            if n.getFval() < tentativeNextCell.getFval():
                tentativeNextCell = n
        publishCell(selectedCell.getXpos(), selectedCell.getYpos(), 'unexplored')
        selectedCell = tentativeNextCell
    path.append(selectedCell)
    print path


def unexploredNeighbors(selectedCell, explored, costMap):
    # iterating through all the cells adjacent to the selected cell
    neighbors_tmp = []
    neighbors = []
    neighbors_tmp.append(costMap[selectedCell.getXpos() + 1][selectedCell.getYpos()])
    neighbors_tmp.append(costMap[selectedCell.getXpos()][selectedCell.getYpos() + 1])
    neighbors_tmp.append(costMap[selectedCell.getXpos() - 1][selectedCell.getYpos()])
    neighbors_tmp.append(costMap[selectedCell.getXpos()][selectedCell.getYpos() - 1])
    # neighbors_tmp.append(costMap[selectedCell.getXpos() - 1][selectedCell.getYpos() - 1])
    # neighbors_tmp.append(costMap[selectedCell.getXpos() + 1][selectedCell.getYpos() + 1])
    # neighbors_tmp.append(costMap[selectedCell.getXpos() - 1][selectedCell.getYpos() + 1])
    # neighbors_tmp.append(costMap[selectedCell.getXpos() + 1][selectedCell.getYpos() - 1])
    for n in neighbors_tmp:
        if n != selectedCell and n not in explored and n.isEmpty():
            n.setParent(selectedCell)
            neighbors.append(n)
    return neighbors


def main():
    DBPrint('main')
    global vel_pub, odom_list

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
    rospy.Subscriber('/navgoal', PoseStamped, goalHandler)

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
