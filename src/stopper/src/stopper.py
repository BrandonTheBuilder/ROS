#!/usr/bin/env python

# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.
import roslib; roslib.load_manifest('stopper')
import rospy
import math
import tf

# The laser scan message
from sensor_msgs.msg import LaserScan

# Odometry message
from nav_msgs.msg import Odometry

# The velocity command message
from geometry_msgs.msg import Twist

# instantiate odometry variable globalOdom
globalOdom = Odometry()


def laser_callback(scan):
    # Make a new Twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified until they are
    # actually assigned.  The Twist message holds linear and angular
    # velocities.
    command.linear.x = 1.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    # import laser scan data
    rangeArray = scan.ranges
    import IPython; IPython.embed()
    # if no obstacle is closer than one meter, move forward
    if min(rangeArray) < 2.0:
        command.angular.x = 0.0
        command.angular.z = 1.0
        command.angular.y = 3.0
        command.angular.z = 3.0

    # publish the command
    pub.publish(command)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('stopper', log_level=rospy.DEBUG)

    # subscribe to base scan messages
    sub = rospy.Subscriber('base_scan', LaserScan, laser_callback)

    # publish twist messages
    pub = rospy.Publisher('cmd_vel', Twist)

    # Turn control over to ROS
    rospy.spin()
