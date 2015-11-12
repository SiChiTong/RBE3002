import rospy, tf, time, math, roslib
from kobuki_msgs.msg import BumperEvent

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from std_msgs.msg import Empty

from tf.transformations import euler_from_quaternion

DEBUG = 0

#Odometry Callback function.
def odomHandler(msg):
    DBPrint("odomHandler")
    global pose, x, y, theta
    pose = msg.pose

    # Update the x, y, and theta globals every time something changes
    quat = pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    x = pose.pose.position.x
    y = pose.pose.position.y
    theta = yaw 

def goalHandler(msg):
    DBPrint('goalHandler')
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
    print msg
   '
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
    ' 

def publishTile(x, y, state):
    DBPrint('publishTile')
    global fully_explored_cells, known_cells, path_cells

    if state == 'fully_explored':
        cells.append()
    elif state == 'known':
        cells.append()
    elif state == 'path':
        cells.append()


    
    
def DBPrint(param):
    if DEBUG == 1:
        print param

def main():
    DBPrint('main')

    rospy.init_node('lab3')

    # Publisher for commanding robot motion
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    
    # Subscribe to bumper changes
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperHandler, queue_size=1) 
    
    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odomHandler)

    # Subscribe to the map
    rospy.Subscriber('/map', OccupancyGrid, mapHandler)

    # Subscribe to NavToGoal stuff
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalHandler)

    # Create tile publishers
    rospy.Publisher('/fully_explored_cells', GridCells, queue_size=1)
    rospy.Publisher('/known_cells', GridCells, queue_size=1)
    rospy.Publisher('/path_cells', GridCells, queue_size=1)
    

    # Create Odemetry listener and boadcaster 
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()

    # Set the transform from base_footprint to odom
    # odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"odom","map")
    
    # Wait for an odom event to init pose
    rospy.sleep(rospy.Duration(1))
    
    rospy.spin()


if __name__ == '__main__':
    main()
