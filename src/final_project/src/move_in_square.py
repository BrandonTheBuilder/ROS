#!/usr/bin/env/python

# import libraries
import roslib
import rospy
import math

# The velocity command message
from geometry_msgs.msg import Twist

def cmdvel_callback(vel):
    #Check if robot has stopped
    if vel.linear.x == 0.0 and vel.linear.y == 0.0 and vel.angular.z == 0.0:
        # Make a new Twist waypoint message
        waypoint = Twist()
        # make a new waypoint 20 units to the right of current location
        waypoint.linear.x = 0.0
        waypoint.linear.y = -20.0
        waypoint.linear.z = 0.0
        # Then rotate 90 degrees
        waypoint.angular.x = 0.0
        waypoint.angular.y = 0.0
        waypoint.angular.z = -90.0

        pub.publish(waypoint)


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('move_in_square')
    # Publish waypoint data to robot
    pub = rospy.Publisher('base_link_goal', Twist, queue_size=10)
    sub = rospy.Subscriber('/cmd_vel', Twist, cmdvel_callback)

    # Turn control back to ros
    rospy.spin()