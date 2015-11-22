#!/usr/bin/python
import rospy
import math

import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion


def nav_path_handler(path_msg):
    """
    Navigate along a path.
    :param path_msg: The path to navigate along.
    """
    for p in path_msg.poses:
        nav_to_pose(p.pose.position.x, p.pose.position.y, math.degrees(p.pose.orientation.z))


def nav_to_pose(goal_x, goal_y, goal_theta):
    """drive to a goal subscribed as /move_base_simple/goal.
    Moves to a pose in the world frame.
    :param goal_x: The goal X position.
    :param goal_y: The goal Y position.
    :param goal_theta: The goal theta orientation.
    """
    global x, y, theta, odom_list

    # Get the position of the robot in the global frame
    (position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

    # Find the distance and angle between the robot and the goal using global frame coordinates
    distance = math.sqrt((goal_y - position[1]) ** 2 + (goal_x - position[0]) ** 2)
    angle = math.atan2(goal_y - position[1], goal_x - position[0])

    # Rotate towards goal point, drive to it, rotate to final pose
    rotate(math.degrees(angle - theta))
    drive_straight(0.5, distance)
    rotate(math.degrees(goal_theta - theta))


def publish_twist(u, w):
    """Publish a twist message to the robot base.
    :param u: Linear velocity.
    :param w: Angular velocity.
    """
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
    global pose

    start_pose = pose
    displacement = 0
    r = rospy.Rate(10)  # 10hz
    while displacement < distance:
        publish_twist(speed, 0)
        displacement = difference(pose, start_pose)[0]
        r.sleep()


def rotate(angle):
    """Accepts an angle and makes the robot rotate around it.
    :param angle: Angle (degrees)
    """
    global pose, theta

    # Correct angle to be withing -180 - 180
    if angle > 180:
        angle = -(angle - 180)
    elif angle < -180:
        angle = -(angle + 180)

    # Convert degrees to radians
    angle = angle * math.pi / 180

    # Get the current orientation
    current_angle = theta

    # Figure out the beginning and end orientations
    start_angle = current_angle
    end_angle = start_angle + angle

    # Make sure end angle is in range
    if end_angle < -math.pi or end_angle > math.pi:
        end_angle = (-math.pi + (abs(end_angle) % math.pi)) * abs(end_angle) / end_angle

    # Constants
    frequency = 10  # Hz
    precision = 0.02  # Radians = ~1 degree
    p, i = 10.0, 0.1  # PID constants

    # Variables
    error = 100
    last_error = error
    total_error = 0

    # While it has not yet reached the desired position, turn
    r = rospy.Rate(frequency)
    while abs(error) > precision:
        total_error += last_error
        last_error = error
        error = end_angle - current_angle
        w = p * error  # - i*total_error

        # Cap the maximum turning rate
        if w > 1:
            w = 1
        elif w < -1:
            w = -1

        publish_twist(0, w)
        current_angle = theta
        r.sleep()


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
    global x, y, theta, pose
    pose = msg.pose

    try:
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

        # Update the x, y, and theta globals every time something changes
        roll, pitch, yaw = euler_from_quaternion(rot)

        x = trans[0]
        y = trans[1]
        theta = yaw
    except:
        pass


def main():
    """
    The main program function.
    """
    global vel_pub, odom_list
    rospy.init_node('rbe3002_move_node')

    move_path_sub = rospy.Subscriber('/nav_path', Path, nav_path_handler)

    # Publisher for commanding robot motion
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    # Subscribe to Odometry changes
    rospy.Subscriber('/odom', Odometry, odom_handler)

    # Create Odemetry listener and boadcaster
    odom_list = tf.TransformListener()

    rospy.spin()

if __name__ == '__main__':
    main()
