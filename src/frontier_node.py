#!/usr/bin/python
import math
import random

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import Point, PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import GridCells, Odometry, OccupancyGrid
from GridCell import GridCell
from nav_msgs.srv import GetMap
from tf.transformations import euler_from_quaternion, quaternion_from_euler

CELL_WIDTH = 0.3
CELL_HEIGHT = 0.3
expanded_cells = []
wall_cells = []
path_cells = []
frontier_cells = []


def get_centroid(frontier):
    x_sum, y_sum = 0.0, 0.0
    total = len(frontier)

    for cell in frontier:
        x_sum = x_sum + cell.getXpos()
        y_sum = y_sum + cell.getYpos()

    cell_x = int(x_sum / total)
    cell_y = int(y_sum / total)

    cell = costMap[cell_x][cell_y]

    if not cell.isEmpty() or cell.isUnknown():
        cell = get_closest_valid_cell(cell)

    return cell


def get_closest_valid_cell(cell):
    unexplored = []
    explored = [cell]

    unexplored.extend(get_neighboring_cells(cell))

    while True:
        tmp = unexplored.pop(0)

        if tmp in explored:
            continue

        explored.append(tmp)

        if tmp.isEmpty() and not tmp.isUnknown():
            return tmp
        else:
            unexplored.extend(get_neighboring_cells(tmp))


def detect_frontiers():
    """
    Detect frontiers in a map.
    :return: A list of frontiers, where each frontier is a list of connected frontier cells
    """
    global nav_goal, unreachable, centroids
    frontier = []

    # Iterate through all of the cells in the global map and collect all the possible frontier cells
    for row in costMap:
        for cell in row:
            if is_frontier_cell(cell):
                frontier.append(cell)

    # Group the frontier cells into continuous frontiers and return them as a list
    groups = group_frontiers(frontier)

    groups = filter(lambda list: longest_distance(list) >= 10, groups)

    for frontier in groups:
        for cell in frontier:
            publish_cell(cell.getXpos(), cell.getYpos(), "frontier")
    for i in range(10):
        publish_cells()

    # Calculate the centroid of all of the frontiers
    centroids = map(get_centroid, groups)

    for cell in centroids:
        publish_cell(cell.getXpos(), cell.getYpos(), "path")
    publish_cells()

    # Calculate the number of frontier cells in each frontier
    lengths = map(len, groups)

    # Calculate the distance to each centroid
    distances = map(distance_to_centroid, centroids)

    # Weight each centroid by its distance * # of frontier cells
    weighted_centroid = []
    try:
        for i in range(len(distances)):
            weighted_centroid.append(lengths[i] / distances[i])
    except ZeroDivisionError:
        print "An Distances to centroids is 0 for some reason!"

    maximum, index = 0, 0
    for i in range(len(weighted_centroid)):
        if weighted_centroid[i] > maximum:
            maximum = weighted_centroid[i]
            index = i

    # The most heavily weighted centroid
    if not unreachable:
        old_nav_goal = nav_goal
        nav_goal = centroids[index]
        if old_nav_goal != nav_goal and nav_goal is not None:  # Change goal if better goal appears.
            cancel_navigation()
            print "Better goal available, canceling current..."
    else:
        nav_goal = centroids[random.randint(0, len(centroids)-1)]
        unreachable = False

    print "Nav Goal " + str(nav_goal)

    publish_cell(nav_goal.getXpos(), nav_goal.getYpos(), "expanded")
    publish_cell(nav_goal.getXpos() + 1, nav_goal.getYpos(), "expanded")
    publish_cell(nav_goal.getXpos() - 1, nav_goal.getYpos(), "expanded")
    publish_cell(nav_goal.getXpos(), nav_goal.getYpos() + 1, "expanded")
    publish_cell(nav_goal.getXpos(), nav_goal.getYpos() - 1, "expanded")
    for i in range(10):
        publish_cells()


def distance_to_centroid(centroid_cell):
    cell_x, cell_y = map_to_grid(x, y)
    return math.sqrt((centroid_cell.getXpos() - cell_x) ** 2 + (centroid_cell.getYpos() - cell_y) ** 2)


def get_neighboring_cells(cell):
    """
    :param cell: The cell, whose neighbors are needed
    :return: A list of cells neightobring cell
    """
    neighbors = []

    x_pos = cell.getXpos()
    y_pos = cell.getYpos()

    # Iterate through the 8 adjacent cells
    for row in range(y_pos - 1, y_pos + 2):
        for col in range(x_pos - 1, x_pos + 2):
            # Filter out bad cells
            if row < 0 or col < 0 or (row == y_pos and col == x_pos):
                continue
            else:
                try:
                    neighbors.append(costMap[col][row])
                except:  # Index out of bounds exception
                    pass

    return neighbors


def has_known_neighbor(neighbors):
    for cell in neighbors:
        if not cell.isUnknown():
            return True

    return False


def is_frontier_cell(cell):
    return cell.isUnknown() and has_known_neighbor(filter(lambda c: c.isEmpty(), get_neighboring_cells(cell)))


def group_frontiers(ungrouped_frontier):
    frontiers = []

    while len(ungrouped_frontier) != 0:
        frontier = [ungrouped_frontier.pop(0)]
        done = False

        # Just ask Tucker what's going on here
        while not done:
            try:
                for a in ungrouped_frontier:
                    for b in frontier:
                        if is_adjacent(a, b):
                            frontier.append(a)
                            ungrouped_frontier.remove(a)
                            raise StopIteration  # Using this to break out of outer for loop
                # Finished finding the frontier. Add it to the frontiers list.
                done = True
                frontiers.append(frontier)
            except:  # Using this as a break loop statement
                pass

    return frontiers


def is_adjacent(a, b):
    return (abs(a.getXpos() - b.getXpos()) <= 1) and (abs(a.getYpos() - b.getYpos()) <= 1)


def odom_handler(msg):
    """
    Odometry callback function.
    :param msg: The odom message.
    """
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
def longest_distance(frontier):
    max_dist = 0

    for a in frontier:
        for b in frontier:
            dist = math.sqrt((a.getXpos() - b.getXpos())**2 + (a.getYpos() - b.getYpos())**2)
            if dist > max_dist:
                max_dist = dist
    return max_dist

def request_map(event):
    """
    Request a map from the costmap service.
    """
    get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)
    map_handler(get_map_srv().map)


def map_handler(msg):
    """
    Handles when a new global map message arrives.
    :param msg: The map message to process.
    """
    print "******************************Got global map******************************"
    global CELL_WIDTH, CELL_HEIGHT
    global map_width, map_height, occupancyGrid, x_offset, y_offset
    global map_origin_x, map_origin_y, costMap
    global path_cells, wall_cells, frontier_cells, expanded_cells

    map_width = msg.info.width
    map_height = msg.info.height
    occupancyGrid = msg.data
    print "Map (width, height): " + str(map_width) + " " + str(map_height)

    CELL_WIDTH = msg.info.resolution
    CELL_HEIGHT = msg.info.resolution

    x_offset = msg.info.origin.position.x + (2 * CELL_WIDTH)
    y_offset = msg.info.origin.position.y - (2 * CELL_HEIGHT)
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    print "Map origin: ", map_origin_x, map_origin_y

    # Create the costMap
    costMap = [[0 for j in range(map_height)] for j in range(map_width)]

    count = 0
    # iterating through every position in the matrix
    # OccupancyGrid is in row-major order
    # Items in rows are displayed in contiguous memory
    path_cells, wall_cells, frontier_cells, expanded_cells = [], [], [], []
    for y_tmp in range(0, map_height):  # Rows
        for x_tmp in range(0, map_width):  # Columns
            costMap[x_tmp][y_tmp] = GridCell(x_tmp, y_tmp, occupancyGrid[count])  # creates all the gridCells
            count += 1
    expand_objects()
    publish_walls()
    detect_frontiers()


def expand_objects():
    global costMap
    for y_tmp in range(0, map_height):  # Rows
        for x_tmp in range(0, map_width):  # Columns
            if costMap[x_tmp][y_tmp].getOccupancyLevel() > 90:
                for y_tmp_2 in range (y_tmp - 5, y_tmp + 6):
                    for x_tmp_2 in range (x_tmp - 5, x_tmp + 6):
                        if math.sqrt((y_tmp_2 - y_tmp)**2 + (x_tmp_2 - x_tmp)**2) > 5:
                            continue
                        try:
                            costMap[x_tmp_2][y_tmp_2].setOccupancyLevel(60)
                        except IndexError:
                            pass


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


def publish_cell(x_position, y_position, state):
    """
    Creates and adds a cell-location to its corresponding list to be published in the near future
    :param x_position: The X position (in terms of cell number) of the cell to color.
    :param y_position: The Y position (in terms of cell number) of the cell to color.
    :param state: The cell "state" to fill in, either expanded, wall, path, or frontier
    """
    global expanded_cells, frontier_cells, wall_cells, path_cells

    p = Point()
    p.x, p.y = map_to_world(x_position, y_position)
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
    publish_expanded()
    publish_frontier()
    publish_walls()
    publish_path()


def publish_expanded():
    """
    Publishes the information stored in expanded_cells to the map
    """
    global pub_expanded

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
    global pub_walls, costMap, wall_cells

    wall_cells = []
    for y_tmp in range(0, map_height):
        for x_tmp in range(0, map_width):
            if not costMap[x_tmp][y_tmp].isEmpty():
                publish_cell(x_tmp, y_tmp, 'wall')

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = wall_cells
    pub_walls.publish(msg)


def publish_path():
    """
    Publishes the information stored in unexplored_cells to the map
    """
    global pub_path

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = path_cells
    pub_path.publish(msg)


def publish_frontier():
    """
    Publishes the information stored in unexplored_cells to the map
    """
    global pub_frontier

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = frontier_cells
    pub_frontier.publish(msg)


def nav_to_pose(goal):
    """
    Drive to a goal subscribed to from /move_base_simple/goal
    :param goal: Goal pose.
    """
    global move_base
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose = goal.pose
    move_base.send_goal(goal_pose)


def cancel_navigation():
    """
    Cancel all goals on the move base.
    """
    cancel = GoalID()
    move_base_cancel.publish(cancel)


def move_status_handler(msg):
    """
    Handle movement status messages.
    :param msg: The move status message array.
    """
    global last_active_goal, goal_done, unreachable
    for goal in msg.status_list:
        if goal.status < 2:
            last_active_goal = goal
            goal_done = False
    if len(msg.status_list) > 0 and not goal_done:
        for goal in msg.status_list:
            if goal.goal_id.id == last_active_goal.goal_id.id:
                if 2 <= goal.status <= 3:
                    goal_done = True
                    print "Goal completed, moving to next centroid."
                    go_to_next_centroid()
                elif 4 <= goal.status <= 5:  # Goal unreachable or rejected
                    goal_done = True
                    print "Goal unreachable or other error."
                    unreachable = True
                    cancel_navigation()
                    go_to_next_centroid()


def go_to_next_centroid():
    nav_goal_pose = PoseStamped()
    nav_goal_pose.header.frame_id = 'map'
    nav_goal_pose.header.stamp = rospy.Time().now()
    world_x, world_y = map_to_world(nav_goal.getXpos(), nav_goal.getYpos())
    nav_goal_pose.pose.position.x = world_x
    nav_goal_pose.pose.position.y = world_y
    # TODO: Turn towards frontier
    nav_goal_pose.pose.orientation.w, nav_goal_pose.pose.orientation.x, nav_goal_pose.pose.orientation.y, nav_goal_pose.pose.orientation.z = quaternion_from_euler(0.0, 0.0, 0.0)
    print "Navigating to centroid..."
    nav_to_pose(nav_goal_pose)


def check_goal(event):
    """
    Check if the robot is near its goal.
    :param event: Timer event.
    """
    global x_cell, y_cell, nav_goal
    x_goal_cell, y_goal_cell = nav_goal.getYpos(), nav_goal.getYpos()
    print x_goal_cell, y_goal_cell, x_cell, y_cell
    print math.sqrt((x_goal_cell - x_cell) ** 2 + (y_goal_cell - y_cell) ** 2)
    if math.sqrt((x_goal_cell - x_cell) ** 2 + (y_goal_cell - y_cell) ** 2) <= 5:
        print "Close enough..."
        cancel_navigation()
        go_to_next_centroid()


if __name__ == '__main__':
    global odom_list, pub_walls, pub_expanded, pub_path, pub_frontier, last_map, move_base_cancel
    global goal_done, unreachable, nav_goal
    rospy.init_node('rbe_3002_frontier_node')
    goal_done = True
    unreachable = False
    nav_goal = None

    move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    pub_walls = rospy.Publisher('/wall_cells', GridCells, queue_size=1)
    pub_expanded = rospy.Publisher('/expanded_cells', GridCells, queue_size=1)
    pub_path = rospy.Publisher('/path_cells', GridCells, queue_size=1)
    pub_frontier = rospy.Publisher('/frontier_cells', GridCells, queue_size=1)

    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odom_handler)

    # Subscribe to the navgoal.
    rospy.Subscriber('/navgoal', PoseStamped, nav_to_pose)

    # Create Odemetry listener and boadcaster
    odom_list = tf.TransformListener()

    # Configure move base action library.
    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(5))
    # Subscribe to move base status.
    move_base_status = rospy.Subscriber('/move_base/status', GoalStatusArray, move_status_handler)

    # Request the global costmap every 5 seconds
    last_map = []

    # Subscribe to the local map
    # rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, local_map_handler)

    request_map(None)
    rospy.sleep(rospy.Duration(5))
    go_to_next_centroid()
    rospy.Timer(rospy.Duration(5), request_map)
    rospy.Timer(rospy.Duration(1), check_goal)

    rospy.spin()
