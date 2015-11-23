#!/usr/bin/python
import rospy
import numpy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

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


def send_move_msg(linear_vel, angular_vel):
    """
    Send a movement (twist) message.
    :param linear_vel: The forward linear robot velocity.
    :param angular_vel: The robot angular velocity.
    """
    global vel_pub
    msg = Twist()
    msg.linear.x = linear_vel
    msg.angular.z = angular_vel
    vel_pub.publish(msg)


def nav_to_pose(goal):
    """
    Drive to a goal subscribed to from /move_base_simple/goal
    :param goal: Goal pose.
    """
    # compute angle required to make straight-line move to desired pose
    global x
    global y
    global theta
    # capture desired x and y positions
    desiredy = goal.pose.position.y
    desiredx = goal.pose.position.x
    # capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredt = yaw * (180.0 / math.pi)
    # compute distance to target
    distance = math.sqrt(math.pow((desiredx - x), 2) + math.pow((desiredy - y), 2))
    adjustedx = goal.pose.position.x - x
    adjustedy = goal.pose.position.y - y
    print goal.pose.position.x, goal.pose.position.y
    print x, y
    print adjustedx, adjustedy
    # compute initial turn amount
    initialturn = (math.atan2(adjustedy, adjustedx) * (180 / math.pi)) - theta

    print "moving from (" + str(x) + ", " + str(y) + ") @ " + str(theta) + " degrees"
    print "moving to (" + str(desiredx) + ", " + str(desiredy) + ") @ " + str(desiredt) + " degrees"
    print "distance: " + str(distance) + ", initial turn: " + str(initialturn)
    print "spin!"  # turn to calculated angle
    rotate_degrees(initialturn)
    print "move!"  # move in straight line specified distance to new pose
    drive_smooth(0.25, distance)
    rospy.sleep(2)
    print "spin!"  # spin to final angle
    finalturn = desiredt - theta
    if finalturn > 180:
        finalturn -= 360
    print "rotate " + str(finalturn) + " to " + str(desiredt)
    rotate_degrees(finalturn)
    print "done"


def rotate_degrees(angle):
    """
    Rotate and angle in degrees.
    :param angle: The angle to rotate (in degrees).
    """
    rotate(angle * (math.pi / 180))


def drive_smooth(speed, distance):
    """
    This function accepts a speed and a distance for the robot to move in a smoothed straight line.
    :param speed: Linear speed.
    :param distance: Distance to move.
    """
    global pose

    initialx = pose.pose.position.x
    initialy = pose.pose.position.y
    attarget = False
    rampspeed = 0.0
    sleeptime = 0.05
    ramppercentage = 0.3
    step = speed / ((ramppercentage * (distance / speed)) / sleeptime)
    print "Step size: " + str(step)
    while not attarget and not rospy.is_shutdown():
        currentx = pose.pose.position.x
        currenty = pose.pose.position.y
        currentdistance = math.sqrt(math.pow((currentx - initialx), 2) + math.pow((currenty - initialy), 2))
        if currentdistance >= distance:
            attarget = True
            send_move_msg(0, 0)
        else:
            if (distance - currentdistance) <= distance * ramppercentage and rampspeed >= 0:
                rampspeed -= step
                send_move_msg(rampspeed, 0)
            elif (distance - currentdistance) >= distance * (1.0 - ramppercentage) and rampspeed <= speed:
                rampspeed += step
                send_move_msg(rampspeed, 0)
            else:
                send_move_msg(speed, 0)
            rospy.sleep(sleeptime)


def rotate(angle):
    """
    Accepts an angle and makes the robot rotate around it.
    :param angle: The angle (in radians) to turn.
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
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    t_o_t = transformer.fromTranslationRotation(trans, rot)
    r_o_t = t_o_t[0:3, 0:3]

    # Setup goal matrix
    goal_rot = numpy.dot(rotation, r_o_t)
    goal_o = numpy.array([[goal_rot[0, 0], goal_rot[0, 1], goal_rot[0, 2], t_o_t[0, 3]],
                          [goal_rot[1, 0], goal_rot[1, 1], goal_rot[1, 2], t_o_t[1, 3]],
                          [goal_rot[2, 0], goal_rot[2, 1], goal_rot[2, 2], t_o_t[2, 3]],
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
    :param time: Time to move.
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


def read_odom(msg):
    """
    Read odometry messages and store into global variables.
    :param msg: The odom message from the callback.
    """
    global pose
    global x
    global y
    global theta
    global odom_list
    try:
        pose = msg.pose
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
        roll, pitch, yaw = euler_from_quaternion(rot)
        theta = yaw * (180.0 / math.pi)
        x = trans[0]
        y = trans[1]
    except:
        print "Waiting for tf..."


def main():
    """
    The main program function.
    """
    global vel_pub, odom_list, odom_sub, move_status_pub
    rospy.init_node('rbe3002_move_node')

    odom_list = tf.TransformListener()
    odom_sub = rospy.Subscriber('/odom', Odometry, read_odom)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    rospy.Subscriber('/nav_path', Path, nav_path_handler)
    move_status_pub = rospy.Publisher('/movement_state', Bool, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
