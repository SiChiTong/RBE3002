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

expanded_cells = []; wall_cells = []; path_cells = []; frontier_cells = []

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
        pass

def goalHandler(msg):
    DBPrint('goalHandler')
    global goal_x, goal_y, goal_theta, x_goal_cell, y_goal_cell, path_cells, expanded_cells, frontier_cells
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

    path_cells = []; expanded_cells = []; frontier_cells = []
    publishPath(); publishExpanded(); publishFrontier()

    AStar()


def mapHandler(msg):
    DBPrint('mapHandler')
    global expanded_cells, frontier_cells, unexplored_cells, CELL_WIDTH, CELL_HEIGHT
    global map_width, map_height, occupancyGrid, x_offset, y_offset

    map_width = msg.info.width
    map_height = msg.info.height
    occupancyGrid = msg.data

    CELL_WIDTH = msg.info.resolution
    CELL_HEIGHT = msg.info.resolution

    x_offset = msg.info.origin.position.x + (2*CELL_WIDTH) 
    y_offset = msg.info.origin.position.y - (2*CELL_HEIGHT)

    index = 0

    for y in range(1, map_height+1):
        for x in range(1, map_width+1):
            index = (y - 1) * map_width + (x - 1)
            if occupancyGrid[index] == 100:
                publishCell(x + x_offset, y + y_offset, 'wall')


# Creates and adds a cell-location to its corresponding list to be published in the near future
def publishCell(x, y, state):
    DBPrint('publishCell')
    global expanded_cells, frontier_cells, wall_cells, path_cells

    p = Point()
    p.x = x*CELL_WIDTH
    p.y = y*CELL_HEIGHT
    p.z = 0

    if state == 'expanded':
        expanded_cells.append(p)
    elif state == 'wall':
        wall_cells.append(p)
    elif state == 'path':
        path_cells.append(p)
    elif state == 'frontier':
        frontier_cells.append(p)
    else:
        print 'Bad state'

# Publishes a message to display all of the cells
def publishCells():
    DBPrint('publishCells')
    publishExpanded()
    publishFrontier()
    publishWalls()
    publishPath()

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
def publishWalls():
    DBPrint('publishWalls')
    pub_frontier = rospy.Publisher('/wall_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = wall_cells
    pub_frontier.publish(msg)

# Publishes the information stored in unexplored_cells to the map
def publishPath():
    DBPrint('publishPath')
    pub_unexplored = rospy.Publisher('/path_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = path_cells
    pub_unexplored.publish(msg)

# Publishes the information stored in unexplored_cells to the map
def publishFrontier():
    DBPrint('publishFrontier')
    pub_unexplored = rospy.Publisher('/frontier_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    
    msg.cells = frontier_cells
    pub_unexplored.publish(msg)

    
def DBPrint(param):
    if DEBUG == 1:
        print param

def AStar():
    global x_cell, y_cell, x_goal_cell, y_goal_cell, frontier_cells, expanded_cells

    # Create the costMap
    costMap = [[0 for x in range(map_width)] for x in range(map_height)]

    count = 0
    # iterating through every position in the matrix
    # OccupancyGrid is in row-major order
    # Items in rows are displayed in contiguous memory
    for y in range (0, map_height): # Rows
        for x in range(0, map_width): # Columns
            costMap[x][y] = GridCell(x, y, occupancyGrid[count]) # creates all the gridCells
            costMap[x][y].setH(x_goal_cell, y_goal_cell) # adds an H value to every gridCell
            count += 1

    # Keep track of explored cells
    open_list = []
    # Keep track of cells with parents
    closed_list = []
    # Keep track of path
    path = []

    # make the start position the selected cell
    selectedCell = costMap[x_cell][y_cell]

    while (selectedCell != costMap[x_goal_cell][y_goal_cell]): 
        closed_list.append(selectedCell)
        neighbors = unexploredNeighbors(selectedCell, open_list, closed_list, costMap)
        open_list.extend(neighbors)
        candidate = open_list[0]
        for cell in open_list:
            if cell.getFval() < candidate.getFval():
                candidate = cell
        selectedCell = candidate
        open_list.remove(selectedCell)
        frontier_cells = []; expanded_cells = [];
        for cell in open_list:
            if cell not in closed_list:
                publishCell(cell.getXpos() + x_offset + 1, cell.getYpos() + y_offset + 1, 'expanded')
        for cell in closed_list:
            publishCell(cell.getXpos() + x_offset + 1, cell.getYpos() + y_offset + 1, 'frontier')
        publishExpanded()
        publishFrontier()

    path_cell = selectedCell
    while path_cell != costMap[x_cell][y_cell]:
        path.append(path_cell)
        path_cell = path_cell.getParent()
    path = list(reversed(path))
    print path
    waypoints = getWaypoints(path)
    for p in path:
        publishCell(p.getXpos() + x_offset + 1, p.getYpos() + y_offset + 1, 'path')
    publishPath()

def unexploredNeighbors(selectedCell, openList, closedList, costMap):
    # iterating through all the cells adjacent to the selected cell
    neighbors_tmp = []
    neighbors = []
    neighbors_tmp.append(costMap[selectedCell.getXpos() + 1][selectedCell.getYpos()])
    neighbors_tmp.append(costMap[selectedCell.getXpos()][selectedCell.getYpos() + 1])
    neighbors_tmp.append(costMap[selectedCell.getXpos() - 1][selectedCell.getYpos()])
    neighbors_tmp.append(costMap[selectedCell.getXpos()][selectedCell.getYpos() - 1])
    neighbors_tmp.append(costMap[selectedCell.getXpos() - 1][selectedCell.getYpos() - 1])
    neighbors_tmp.append(costMap[selectedCell.getXpos() + 1][selectedCell.getYpos() + 1])
    neighbors_tmp.append(costMap[selectedCell.getXpos() - 1][selectedCell.getYpos() + 1])
    neighbors_tmp.append(costMap[selectedCell.getXpos() + 1][selectedCell.getYpos() - 1])
    for n in neighbors_tmp:
        if (n != selectedCell) and (n not in openList) and (n not in closedList) and n.isEmpty():
            n.setParent(selectedCell)
            neighbors.append(n)
    return neighbors

def getWaypoints(path):
    waypoints = []
    direction = []
    posePath = []
    changeX = 0
    changeY = 0
    # checking if the robot changes direction along the path
    for i in range (1, path.size()):
        newdX = path[i].getXpos() - path[i-1].getXpos()
        newdY = path[i].getYpos() - path[i-1].getYpos()
        # if the robot does change direction, record the direction it was facing, and record the position the robot was at
        if newdX != changeX or newdY != changeY:
            direction.extend(getDirection(changeX, changeY))
            waypoints.extend(path[i-1])
        changeX = newdX
        changeY = newdY
    # also record the last position of the robot
    direction.extend(getDirection(changeX, changeY))
    waypoints.extend(path[path.size()-1])
    # turn all the data into a list poses
    for i in range (0, waypoints.size()):
        pose = PoseStamped()
        pose.pose.position.x = waypoints[i].getXpos() * CELL_WIDTH + 2 * CELL_WIDTH
        pose.pose.position.y = waypoints[i].getYpos() * CELL_WIDTH
        pose.pose.orientation.z = direction[i]
        posePath.extend(pose)
    return posePath

def getDirection(x,y):
    if x == -1:
        if y == -1:
            return 3*math.pi/4
        else if y == 0:
            return pi
        else:
            return -3*math.pi/4
    else if x == 0:
        if y == -1:
            return math.pi/2
        else if y == 0:
            return 0
        else
            return -math.pi/2
    else:
        if y == -1:
            return math.pi/4
        else if y == 0:
            return 0
        else
            return -math.pi/4 

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
   
    publishPath()
    publishFrontier()
    publishExpanded()
    # Wait for an odom event to init pose
    rospy.sleep(rospy.Duration(1))
    
    rospy.spin()

if __name__ == '__main__':
    main()
