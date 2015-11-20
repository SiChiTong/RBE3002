#!/usr/bin/python
import math
import numpy
import rospy
import tf
from GridCell import GridCell
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells, Path
from tf.transformations import euler_from_quaternion

DEBUG = 0
CELL_WIDTH = 0.3
CELL_HEIGHT = 0.3

wheel_rad = 3.5 / 100.0  # cm
wheel_base = 23.0 / 100.0  # cm

expanded_cells = []
wall_cells = []
path_cells = []
frontier_cells = []


def nav_to_pose(goal_x, goal_y, goal_theta):
    """drive to a goal subscribed as /move_base_simple/goal.
    Moves to a pose in the world frame.
    :param goal_x: The goal X position.
    :param goal_y: The goal Y position.
    :param goal_theta: The goal theta orientation.
    """
    db_print("navToPose")
    global x, y, theta, odom_list

    # Get the position of the robot in the global frame
    (position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

    # Find the distance and angle between the robot and the goal using global frame coordinates
    distance = math.sqrt((goal_y - position[1]) ** 2 + (goal_x - position[0]) ** 2)
    angle = math.atan2(goal_x - position[0], goal_y - position[1])

    # Rotate towards goal point, drive to it, rotate to final pose
    rotate(math.degrees(angle - theta))
    drive_straight(0.5, distance)
    rotate(math.degrees(goal_theta - theta))


def publish_twist(u, w):
    """Publish a twist message to the robot base.
    :param u: Linear velocity.
    :param w: Angular velocity.
    """
    db_print("publishTwist")

    # Populate message with data
    msg = Twist()
    msg.linear.x = u
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = w

    # Publish the message
    vel_pub.publish(msg)


def drive_straight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line
    :param speed: The forward robot speed in m/s.
    :param distance: The forward distance to move in m.
    """
    db_print("driveStraight")
    global pose

    start_pose = pose
    displacement = 0
    r = rospy.Rate(10)  # 10hz
    while displacement < distance:
        publish_twist(speed, 0)
        displacement = difference(pose, start_pose)[0]
        r.sleep()


def rotate(angle):
    """
    Accepts an angle and makes the robot rotate around it.
    :param angle: The angle in radians to rotate around.
    """
    global odom_list
    global pose

    # This node was created using Coordinate system transforms and numpy arrays.
    # The goal is measured in the turtlebot's frame, transformed to the odom.frame
    transformer = tf.TransformerROS()
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],  # Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0, 0, 1]])

    # Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3, 0:3]

    # Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0, 0], goal_rot[0, 1], goal_rot[0, 2], T_o_t[0, 3]],
                          [goal_rot[1, 0], goal_rot[1, 1], goal_rot[1, 2], T_o_t[1, 3]],
                          [goal_rot[2, 0], goal_rot[2, 1], goal_rot[2, 2], T_o_t[2, 3]],
                          [0, 0, 0, 1]])

    # Continues creating and matching coordinate transforms.
    done = False
    while not done and not rospy.is_shutdown():
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if within_tolerance.all():
            spin_wheels(0, 0, 0)
            done = True
        else:
            if angle > 0:
                spin_wheels(1, -1, .1)
            else:
                spin_wheels(-1, 1, .1)


def spin_wheels(u1, u2, time):
    """
    This function accepts two wheel velocities and a time interval.
    :param u1: Wheel 1 speed.
    :param u2: Wheel 2 speed.
    :param time: Movement time.
    """
    global vel_pub

    r = wheel_rad
    b = wheel_base
    # compute wheel speeds
    u = (r / 2) * (u1 + u2)
    w = (r / b) * (u1 - u2)
    start = rospy.Time().now().secs
    # create movement and stop messages
    move_msg = Twist()
    move_msg.linear.x = u
    move_msg.angular.z = w
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    # publish move message for desired time
    while rospy.Time().now().secs - start < time and not rospy.is_shutdown():
        vel_pub.publish(move_msg)
    vel_pub.publish(stop_msg)


def difference(p1, p2):
    """Function that determines the difference in two poses
    :param p1: Starting pose
    :param p2: Ending pose.
    :return: Returns (displacement, delta_x, delta_y, delta_z, delta_w)
    """
    return [
        math.sqrt((p1.pose.position.x - p2.pose.position.x) ** 2 +
                  (p1.pose.position.y - p2.pose.position.y) ** 2),

        # p1.pose.orientation.x - p2.pose.orientation.x,
        # p1.pose.orientation.y - p2.pose.orientation.y,
        # p1.pose.orientation.z - p2.pose.orientation.z,
        # p1.pose.orientation.w - p2.pose.orientation.w
    ]


def odom_handler(msg):
    """Odometry callback function.
    :param msg: The odom message.
    """
    db_print("odomHandler")
    global x, y, theta, x_cell, y_cell, pose
    pose = msg.pose

    try:
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

        # Update the x, y, and theta globals every time something changes
        roll, pitch, yaw = euler_from_quaternion(rot)

        x = trans[0]
        y = trans[1]
        theta = yaw
        x_cell, y_cell = map_to_grid(x, y)
    except:
        pass


def goal_handler(msg):
    """
    Handles when a new move goal is received from Rviz.
    :param msg: The new desired pose (in the world frame) from Rviz.
    """
    db_print('goalHandler')
    global goal_x, goal_y, goal_theta, x_goal_cell, y_goal_cell, path_cells, expanded_cells, frontier_cells
    pose = msg.pose

    # Set the goal_x, goal_y, and goal_theta variables
    quat = pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    goal_x = pose.position.x
    goal_y = pose.position.y
    goal_theta = yaw

    x_goal_cell, y_goal_cell = map_to_grid(goal_x, goal_y)

    print 'Goal ', x_goal_cell, y_goal_cell
    print 'Start ', x_cell, y_cell

    path_cells = []
    expanded_cells = []
    frontier_cells = []
    publish_path()
    publish_expanded()
    publish_frontier()

    pose_tmp = PoseStamped()
    pose_tmp.pose.position.x = x
    pose_tmp.pose.position.y = y
    pose_tmp.pose.orientation.z = theta
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.poses.append(pose_tmp)
    pub_path.publish(path_msg)

    path = astar(x_cell, y_cell, x_goal_cell, y_goal_cell)
    for p in path.poses:
        nav_to_pose(p.pose.position.x, p.pose.position.y, math.degrees(p.pose.orientation.z))


def map_handler(msg):
    """
    Handles when a new map message arrives.
    :param msg: The map message to process.
    """
    db_print('mapHandler')
    global expanded_cells, frontier_cells, unexplored_cells, CELL_WIDTH, CELL_HEIGHT
    global map_width, map_height, occupancyGrid, x_offset, y_offset
    global map_origin_x, map_origin_y, costMap

    map_width = msg.info.width
    map_height = msg.info.height
    occupancyGrid = msg.data
    print map_width, map_height

    CELL_WIDTH = msg.info.resolution
    CELL_HEIGHT = msg.info.resolution

    x_offset = msg.info.origin.position.x + (2 * CELL_WIDTH)
    y_offset = msg.info.origin.position.y - (2 * CELL_HEIGHT)
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    print "Map origin: ", map_origin_x, map_origin_y

    for y in range(1, map_height + 1):
        for x in range(1, map_width + 1):
            index = (y - 1) * map_width + (x - 1)
            if occupancyGrid[index] == 100:
                publish_cell(x, y, 'wall')
    publish_walls()

    # Create the costMap
    costMap = [[0 for x in range(map_height)] for x in range(map_width)]

    count = 0
    # iterating through every position in the matrix
    # OccupancyGrid is in row-major order
    # Items in rows are displayed in contiguous memory
    for y in range(0, map_height):  # Rows
        for x in range(0, map_width):  # Columns
            costMap[x][y] = GridCell(x, y, occupancyGrid[count])  # creates all the gridCells
            count += 1


def map_to_grid(global_x, global_y):
    """
    Map a global coordinate to a grid cell position.
    :param global_x: The global X coordinate.
    :param global_y: The global Y coordinate.
    :return: A tuple representing the X, Y coordinate on the grid.
    """
    grid_x = int(math.floor((global_x - map_origin_x) / CELL_WIDTH))
    grid_y = int(math.floor((global_y - map_origin_y) / CELL_HEIGHT))
    return grid_x, grid_y


def map_to_world(grid_x, grid_y):
    """
    Map a grid X and Y to a global X and Y.
    :param grid_x: The grid X position.
    :param grid_y: The grid Y position.
    :return: A tuple representing the X, Y coordinate in the world frame.
    """
    global_x = (grid_x * CELL_WIDTH + map_origin_x) + (CELL_WIDTH / 2)
    global_y = (grid_y * CELL_WIDTH + map_origin_y) + (CELL_HEIGHT / 2)
    return global_x, global_y


def publish_cell(x, y, state):
    """
    Creates and adds a cell-location to its corresponding list to be published in the near future
    :param x: The X position (in terms of cell number) of the cell to color.
    :param y: The Y position (in terms of cell number) of the cell to color.
    :param state: The cell "state" to fill in, either expanded, wall, path, or frontier
    """
    db_print('publishCell')
    global expanded_cells, frontier_cells, wall_cells, path_cells

    p = Point()
    p.x, p.y = map_to_world(x, y)
    # p.x -= CELL_WIDTH / 2
    # p.y -= CELL_HEIGHT / 2
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


def publish_cells():
    """
    Publishes messages to display all cells.
    """
    db_print('publishCells')
    publish_expanded()
    publish_frontier()
    publish_walls()
    publish_path()


def publish_expanded():
    """
    Publishes the information stored in expanded_cells to the map
    """
    db_print('publishExpanded')
    pub_expanded = rospy.Publisher('/expanded_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = expanded_cells
    pub_expanded.publish(msg)


def publish_walls():
    """
    Publishes the information stored in frontier_cells to the map
    """
    db_print('publishWalls')
    pub_frontier = rospy.Publisher('/wall_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = wall_cells
    pub_frontier.publish(msg)


def publish_path():
    """
    Publishes the information stored in unexplored_cells to the map
    """
    db_print('publishPath')
    pub_unexplored = rospy.Publisher('/path_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = path_cells
    pub_unexplored.publish(msg)


def publish_frontier():
    """
    Publishes the information stored in unexplored_cells to the map
    """
    db_print('publishFrontier')
    pub_unexplored = rospy.Publisher('/frontier_cells', GridCells, queue_size=1)

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = frontier_cells
    pub_unexplored.publish(msg)


def db_print(param):
    """
    Prints out a message if debugging is enabled.
    :param param: The message to print.
    """
    if DEBUG == 1:
        print param


def astar(x_cell, y_cell, x_goal_cell, y_goal_cell):
    """
    Complete the A* path planning algorithm on the globally stored map.
    :param x_cell: The starting X cell on the map.
    :param y_cell: The starting Y cell on the map.
    :param x_goal_cell: The goal X cell on the map.
    :param y_goal_cell: The goal Y cell on the map.
    :return: The path from the starting pose to the ending pose planned by the algorithm.
    """
    global frontier_cells, expanded_cells

    # Keep track of explored cells
    open_list = []
    # Keep track of cells with parents
    closed_list = []
    # Keep track of path
    path = []

    # make the start position the selected cell
    selectedCell = costMap[x_cell][y_cell]

    for y in range(0, map_height):  # Rows
        for x in range(0, map_width):  # Columns
            costMap[x][y].setH(x_goal_cell, y_goal_cell)  # adds an H value to every gridCell

    while selectedCell != costMap[x_goal_cell][y_goal_cell]:
        closed_list.append(selectedCell)
        neighbors = unexplored_neighbors(selectedCell, open_list, closed_list, costMap)
        open_list.extend(neighbors)
        candidate = open_list[0]
        for cell in open_list:
            if cell.getFval() < candidate.getFval():
                candidate = cell
        selectedCell = candidate
        open_list.remove(selectedCell)
        frontier_cells = []
        expanded_cells = []
        for cell in open_list:
            if cell not in closed_list:
                publish_cell(cell.getXpos(), cell.getYpos(), 'expanded')
        for cell in closed_list:
            publish_cell(cell.getXpos(), cell.getYpos(), 'frontier')
        publish_expanded()
        publish_frontier()

    path_cell = selectedCell
    while path_cell != costMap[x_cell][y_cell]:
        path.append(path_cell)
        path_cell = path_cell.getParent()
    path = list(reversed(path))
    print path
    # Publish path
    waypoints = get_local_waypoints(path)
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.poses.extend(waypoints)
    pub_path.publish(path_msg)
    publish_expanded()
    publish_frontier()
    # Send path to gridcells
    for p in path:
        publish_cell(p.getXpos(), p.getYpos(), 'path')
    publish_path()
    return path_msg


def unexplored_neighbors(selectedCell, openList, closedList, costMap):
    """
    Returns a list of all of the unexplored neighbors of a given cell. This method
    also sets all of the neighbor cells parents to the selected cell.
    :param selectedCell: The cell to search for neighbors of.
    :param openList: The open list from A*.
    :param closedList: The closed list from A*.
    :param costMap: The global costmap.
    :return: A list of GridCells containing the eligible neighbor candidates.
    """
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


def astar_handler(req):
    """
    Handles A* requests that come in on the A* service.
    :param req: The service request.
    :return: A path planned by the A* algorithm.
    """
    global goal_x, goal_y, goal_theta, x_goal_cell, y_goal_cell, path_cells, expanded_cells, frontier_cells
    start_pose = req.startPose.pose
    goal_pose = req.endPose.pose

    # Set the goal_x, goal_y, and goal_theta variables
    quat = goal_pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    goal_x = goal_pose.position.x
    goal_y = goal_pose.position.y
    goal_theta = yaw

    x_cell = int((start_pose.position.x - 2 * CELL_WIDTH) // CELL_WIDTH)
    y_cell = int(start_pose.position.y // CELL_WIDTH)

    x_goal_cell = int((goal_x - 2 * CELL_WIDTH) // CELL_WIDTH)
    y_goal_cell = int(goal_y // CELL_WIDTH)

    print 'Goal ', x_goal_cell, y_goal_cell

    path_cells = []
    expanded_cells = []
    frontier_cells = []
    publish_path()
    publish_expanded()
    publish_frontier()

    return astar(x_cell, y_cell, x_goal_cell, y_goal_cell)


def get_waypoints(path):
    """
    Get path waypoints from a list of GridCells defining the path from the
    start to the end of navigation.
    :param path: A list of GridCells defining the path.
    :return: A nav_msgs/Path message containing extracted the path.
    """
    waypoints = []
    direction = []
    posePath = []
    changeX = 0
    changeY = 0
    # checking if the robot changes direction along the path
    for i in range(1, len(path)):
        newdX = path[i].getXpos() - path[i - 1].getXpos()
        newdY = path[i].getYpos() - path[i - 1].getYpos()
        # if the robot does change direction, record the direction it was facing,
        # and record the position the robot was at
        if newdX != changeX or newdY != changeY:
            direction.append(get_direction(changeX, changeY))
            waypoints.append(path[i - 1])
        changeX = newdX
        changeY = newdY
    # also record the last position of the robot
    direction.append(get_direction(changeX, changeY))
    waypoints.append(path[len(path) - 1])
    # turn all the data into a list poses
    for i in range(0, len(waypoints)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = map_to_world(waypoints[i].getXpos(), waypoints[i].getYpos())
        pose.pose.orientation.z = direction[i]
        posePath.append(pose)
    return posePath


def get_local_waypoints(path):
    """
    Get path waypoints from a list of GridCells defining the path from the
    start to the end of navigation. The waypoints are broken up into smaller
    chunks to make it easier for the robot to navigate and re-plan.
    :param path: A list of GridCells defining the path.
    :return: A nav_msgs/Path message containing extracted the path.
    """
    waypoints = []
    direction = []
    posePath = []
    changeX = 0
    changeY = 0
    distance = 0
    # checking if the robot changes direction along the path
    for i in range(1, len(path)):
        newdX = path[i].getXpos() - path[i - 1].getXpos()
        newdY = path[i].getYpos() - path[i - 1].getYpos()
        # if the robot does change direction, record the direction it was facing,
        # and record the position the robot was at
        if newdX != changeX or newdY != changeY:
            direction.append(get_direction(changeX, changeY))
            waypoints.append(path[i - 1])
        else:
            distance += CELL_WIDTH
            if distance >= 1.0:
                direction.append(get_direction(newdX, newdY))
                waypoints.append(path[i])
                distance = 0
        changeX = newdX
        changeY = newdY
    # also record the last position of the robot
    direction.append(get_direction(changeX, changeY))
    waypoints.append(path[len(path) - 1])
    # turn all the data into a list poses
    for i in range(0, len(waypoints)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = map_to_world(waypoints[i].getXpos(), waypoints[i].getYpos())
        pose.pose.orientation.z = direction[i]
        posePath.append(pose)
    return posePath


def get_direction(x, y):
    """
    Get the direction from one cell to the next cell.
    :param x: The change in X coordinates between the previous and current cell.
    :param y: The change in Y coordinates between the previous and current cell.
    :return: An angle, the direction from the previous to the current cell.
    """
    if x == -1:
        if y == -1:
            return 3 * math.pi / 4
        elif y == 0:
            return math.pi
        else:
            return -3 * math.pi / 4
    elif x == 0:
        if y == -1:
            return math.pi / 2
        elif y == 0:
            return 0
        else:
            return -math.pi / 2
    else:
        if y == -1:
            return math.pi / 4
        elif y == 0:
            return 0
        else:
            return -math.pi / 4


def main():
    """
    The main program function.
    """
    db_print('main')
    global vel_pub, odom_list, pub_path, move_base

    rospy.init_node('rbe3002_nav_node')

    # Publisher for publishing the navigation path determined by A*
    pub_path = rospy.Publisher('/nav_path', Path, queue_size=1)

    # Publisher for commanding robot motion
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odom_handler)

    # Subscribe to the map
    rospy.Subscriber('/map', OccupancyGrid, map_handler)

    # Subscribe to NavToGoal stuff
    rospy.Subscriber('/navgoal', PoseStamped, goal_handler)

    # Create Odemetry listener and boadcaster 
    odom_list = tf.TransformListener()

    # Create an A* ros service
    # rospy.Service('A*', aStar, aStarHandler)

    publish_expanded()
    publish_frontier()
    publish_path()
    # Wait for an odom event to init pose
    rospy.sleep(rospy.Duration(1))

    rospy.spin()


if __name__ == '__main__':
    main()
