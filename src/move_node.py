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
        navToPose(p.pose.position.x, p.pose.position.y, p.pose.orientation.z)


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    print "In navToPose"
    global odom_list
    global pose 	
    global pub
    
    x0 = pose.pose.position.x   #Set origin
    y0 = pose.pose.position.y

    transformer = tf.TransformerROS()	
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    state = transformer.fromTranslationRotation(trans, rot)
    theta0 = math.acos(state[1,1]) * 180 / math.pi

    print x0
    print y0
    print theta0

    x1 = goal.pose.position.x
    y1 = goal.pose.position.y
    theta1 = goal.pose.orientation.z

    print x1
    print y1
    print theta1

    rot1 = math.atan((y1-y0) / (x1-x0)) - theta0

    print rot1

    rotate(rot1)

    distance = math.sqrt(math.pow(x1-x0,2) + math.pow(y1-y0,2))
    driveStraight(.25,distance)

    transformer = tf.TransformerROS()	
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    state = transformer.fromTranslationRotation(trans, rot)
    theta0 = math.acos(state[1,1]) * 180 / math.pi
    
    print theta1 - theta0

    rotate(theta1 - theta0)

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


#This function accepts a speed and a distance for the robot to move in a straight line THIS WORKS TOO WOO!!
def driveStraight(speed, distance):
    global odom_list
    global pose 	
    global pub

    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt(math.pow(x1-x0,2) + math.pow(y1-y0,2))
        twist = Twist();
        twist.linear.x = speed
        twist.angular.z = 0
        if (d >= distance):
            done = True
            stopMoving();
        else:
            pub.publish(twist)



def rotate(angle):
    global odom_list
    global pose

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    #Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            stopMoving()
            done = True
        else:
            if (angle > 0):
                spinWheels(.05,-.05,.01)
            else:
                spinWheels(-.05,.05,.01)


#stops the robot
def stopMoving():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
  
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
        (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")


def main():
    """
    The main program function.
    """
    global vel_pub, odom_list
    rospy.init_node('rbe3002_move_node')

   # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    wheel_base = 0.23 #meters    
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    nav_sub = rospy.Subscriber('/move_base_simple/goal/RBE', PoseStamped, navToPose, queue_size=1)
    sub = rospy.Subscriber("/odom", Odometry, readOdom)

    rospy.spin()

if __name__ == '__main__':
    main()
