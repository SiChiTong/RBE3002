#!/usr/bin/python
import math
import rospy
import tf
import time
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells, Odometry, OccupancyGrid
from GridCell import GridCell
from tf.transformations import euler_from_quaternion

CELL_WIDTH = 0.3
CELL_HEIGHT = 0.3
expanded_cells = []
wall_cells = []
path_cells = []
frontier_cells = []


def detect_frontiers():
    frontier = []

    for row in costMap:
        for cell in row:
            if is_frontier_cell(cell):
                frontier.append(cell)

    for cell in frontier:
        publish_cell(cell.getXpos(), cell.getYpos(), "frontier")
    while True:
        publish_cells()
        raw_input("Publish")
        group_frontiers(frontier)
    return frontier


def get_neighboring_cells(cell):
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
                except: # Index out of bounds exception
                    pass

    return neighbors


def has_known_neighbor(neighbors):
    for cell in neighbors:
        if not cell.isUnknown():
            return True

    return False


def is_frontier_cell(cell):
    return cell.isUnknown() and has_known_neighbor(filter(lambda c: c.isEmpty(),get_neighboring_cells(cell)))


def group_frontiers(ungrouped_frontier):
    print "Printing grouped frontiers"
    print len(ungrouped_frontier)
    frontiers = []

    while len(ungrouped_frontier) != 0:
        frontier = [ungrouped_frontier.pop(0)]
        done = False

        # Just ask Tucker what's going on here
        while not done:
            try:
                for a in ungrouped_frontier:
                    for b in frontier:
                        if isAdjacent(a, b):
                            frontier.append(a)
                            ungrouped_frontier.remove(a)
                            raise StopIteration                 # Using this to break out of outer for loop
                # Finished finding the frontier. Add it to the frontiers list.
                done = True
                frontiers.append(frontier)
            except:
                pass

    print len(frontiers)
    return frontiers


def isAdjacent(a, b):
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


def map_handler(msg):
    """
    Handles when a new global map message arrives.
    :param msg: The map message to process.
    """
    global CELL_WIDTH, CELL_HEIGHT
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

    for y_tmp in range(0, map_height):
        for x_tmp in range(0, map_width):
            index = y_tmp * map_width + x_tmp
            if occupancyGrid[index] > 30:
                publish_cell(x_tmp, y_tmp, 'wall')
    publish_walls()

    # Create the costMap
    costMap = [[0 for j in range(map_height)] for j in range(map_width)]

    count = 0
    # iterating through every position in the matrix
    # OccupancyGrid is in row-major order
    # Items in rows are displayed in contiguous memory
    for y_tmp in range(0, map_height):  # Rows
        for x_tmp in range(0, map_width):  # Columns
            costMap[x_tmp][y_tmp] = GridCell(x_tmp, y_tmp, occupancyGrid[count])  # creates all the gridCells
            count += 1

    print detect_frontiers()


def local_map_handler(msg):
    """
    Handles when a new local map message arrives.
    :param msg: The map message to process.
    """
    global CELL_WIDTH, CELL_HEIGHT
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
        for y_tmp in range(y_cell_start, y_cell_start + local_map_height):  # Rows
            for x_tmp in range(x_cell_start, x_cell_start + local_map_width):  # Columns
                costMap[x_tmp][y_tmp].setOccupancyLevel(local_occupancy_grid[count])
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
    pub_unexplored = rospy.Publisher('/frontier_cells', GridCells, queue_size=1)
    print "publishing..."

    # Information all GridCells messages will use
    msg = GridCells()
    msg.header.frame_id = 'map'
    msg.cell_width = CELL_WIDTH
    msg.cell_height = CELL_HEIGHT

    msg.cells = frontier_cells
    pub_unexplored.publish(msg)


if __name__ == '__main__':
    global odom_list
    rospy.init_node('rbe_3002_frontier_node')

    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odom_handler)

    # Subscribe to the global map
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, map_handler)

    # Subscribe to the local map
    rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, local_map_handler)

    # Create Odemetry listener and boadcaster
    odom_list = tf.TransformListener()

    rospy.spin()
