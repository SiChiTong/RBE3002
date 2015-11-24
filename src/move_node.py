#!/usr/bin/python
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import Bool

wheel_rad = 3.5 / 100.0  # cm
wheel_base = 23.0 / 100.0  # cm


def send_move_status(msg):
    """
    Send a movement status message.
    :param msg: A bool, true if moving, false if stopped.
    """
    global move_status_pub
    status_msg = Bool()
    status_msg.data = msg
    move_status_pub.publish(status_msg)


def nav_path_handler(path_msg):
    """
    Navigate along a path.
    :param path_msg: The path to navigate along.
    """
    print "Got path! ", path_msg
    send_move_status(True)
    for p in path_msg.poses:
        nav_to_pose(p)
    send_move_status(False)


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
    success = move_base.wait_for_result(rospy.Duration(60))


def main():
    """
    The main program function.
    """
    global move_status_pub, move_base
    rospy.init_node('rbe3002_move_node')

    rospy.Subscriber('/nav_path', Path, nav_path_handler)
    move_status_pub = rospy.Publisher('/movement_state', Bool, queue_size=1)
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(5))

    rospy.spin()


if __name__ == '__main__':
    main()
