#!/usr/bin/python
import math
import rospy
import tf
from GridCell import GridCell
from geometry_msgs.msg import Point, PoseStamped
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

    astar(x_cell, y_cell, x_goal_cell, y_goal_cell)


def map_handler(msg):
    """
    Handles when a new global map message arrives.
    :param msg: The map message to process.
    """
    global x_goal_cell, y_goal_cell
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

    try:
        if (x_goal_cell != x_cell) and (y_goal_cell != y_cell):
            print "Not at goal, re-planning..."
            print 'Goal ', x_goal_cell, y_goal_cell
            print 'Start ', x_cell, y_cell
            astar(x_cell, y_cell, x_goal_cell, y_goal_cell)
    except NameError:
        print "No goal yet."


def local_map_handler(msg):
    """
    Handles when a new local map message arrives.
    :param msg: The map message to process.
    """
    global x_goal_cell, y_goal_cell
    global expanded_cells, frontier_cells, unexplored_cells, CELL_WIDTH, CELL_HEIGHT
    global map_width, map_height
    global map_origin_x, map_origin_y, costMap

    local_map_width = msg.info.width
    local_map_height = msg.info.height
    local_occupancy_grid = msg.data
    # print "Local dimensions: ", local_map_width, local_map_height

    local_origin_x = msg.info.origin.position.x
    local_origin_y = msg.info.origin.position.y
    try:
        (position, orientation) = odom_list.lookupTransform('odom', 'map', rospy.Time(0))
        local_origin_x += position[0]
        local_origin_y += position[1]
        # print "Map origin: ", local_origin_x, local_origin_y

        # iterating through every position in the matrix
        # OccupancyGrid is in row-major order
        # Items in rows are displayed in contiguous memory
        count = 0
        x_cell_start, y_cell_start = map_to_grid(local_origin_x, local_origin_y)
        for y in range(y_cell_start, y_cell_start + local_map_height):  # Rows
            for x in range(x_cell_start, x_cell_start + local_map_width):  # Columns
                costMap[x][y].setOccupancyLevel(local_occupancy_grid[count]) # update gridCells based on local map
                count += 1
    except:
        print "Map not ready yet."


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

    goal_cell = costMap[x_goal_cell][y_goal_cell]
    if not goal_cell.isEmpty():
        print "No thanks"
        return

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
                direction.append(direction[-1])
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
    global vel_pub, odom_list, pub_path

    rospy.init_node('rbe3002_planning_node')

    # Publisher for publishing the navigation path determined by A*
    pub_path = rospy.Publisher('/nav_path', Path, queue_size=1)

    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odom_handler)

    # Subscribe to the global map
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, map_handler)

    # Subscribe to the local map
    rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, local_map_handler)

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
