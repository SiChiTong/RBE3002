#!/usr/bin/python
import actionlib
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from frontier_node import map_to_grid
from tf.transformations import euler_from_quaternion

<<<<<<< HEAD
=======

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


>>>>>>> cb2d07cac7f35af9956fe7ab6df5aab01733f0bc
def nav_to_pose(goal):
    """
    Drive to a goal subscribed to from /move_base_simple/goal
    :param goal: Goal pose.
    :return: Whether or not the movement was successful.
    """
    global move_base
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose = goal.pose
    move_base.send_goal(goal_pose)
    success = move_base.wait_for_result(rospy.Duration(60))
    return success


def scan():
    """
    Rotate the robot 360 degrees to scan the new area.
    """
    pass


if __name__ == '__main__':
    global move_base, odom_list
    rospy.init_node('rbe_3002_explorer_node')

    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(5))

    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odom_handler)

    # Create Odemetry listener and boadcaster
    odom_list = tf.TransformListener()
