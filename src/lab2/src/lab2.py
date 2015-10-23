#!/usr/bin/env python

# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.

import roslib; roslib.load_manifest('lab2')
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import message_filters

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry

# the velocity command message
from geometry_msgs.msg import Twist

# instantiate global variables "globalOdom"
globalOdom = Odometry()

def polarFromCart(v):
    d = math.pow(math.pow(v[0],2)+math.pow(v[1],2), 0.5)
    theta = math.atan2(v[1], v[0])
    return (d, theta)

# method to control the robot
def callback(scan,odom):
    # the odometry parameter should be global
    global globalOdom
    globalOdom = odom

    # make a new twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    #get goal x and y locations from the launch file
    goalX = rospy.get_param('lab2/goalX',0.0)
    goalY = rospy.get_param('lab2/goalY',0.0)
    
    # find current (x,y) position of robot based on odometry
    currentX = globalOdom.pose.pose.position.x
    currentY = globalOdom.pose.pose.position.y

    # find current orientation of robot based on odometry (quaternion coordinates)
    xOr = globalOdom.pose.pose.orientation.x
    yOr = globalOdom.pose.pose.orientation.y
    zOr = globalOdom.pose.pose.orientation.z
    wOr = globalOdom.pose.pose.orientation.w

    # find orientation of robot (Euler coordinates)
    (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

    # find currentAngle of robot (equivalent to yaw)
    # now that you have yaw, the robot's pose is completely defined by (currentX, currentY, currentAngle)
    currentAngle = yaw

    # find laser scanner properties (min scan angle, max scan angle, scan angle increment)
    maxAngle = scan.angle_max
    minAngle = scan.angle_min
    angleIncrement = scan.angle_increment

    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    currentLaserTheta = minAngle
    maxScanLength = scan.range_max 
    distanceArray = scan.ranges
    numScans = len(distanceArray)
   
    # the code below (currently commented) shows how 
    # you can print variables to the terminal (may 
    # be useful for debugging)
    #print 'x: {0}'.format(currentX)
    #print 'y: {0}'.format(currentY)
    #print 'theta: {0}'.format(currentAngle)

    # for each laser scan
    goal = [goalX-currentX, goalY-currentY]
    

    goalDist, goalAngle = polarFromCart(goal)
    
    print "location {}, {}".format(currentX, currentY)
    print "orientation {}".format(currentAngle)
    print "goal Location {},{}".format(goalX, goalY)
    print "Goal {}, {}".format(goalDist, goalAngle)


    for curScan in range(0, numScans):
        d = distanceArray[curScan]
        if d < 2 and abs(currentLaserTheta) < 1.445:
            goalAngle = currentAngle-math.acos(0.25/d)*currentLaserTheta/abs(currentLaserTheta)
            command.angular.z = (goalAngle - currentAngle)
            command.linear.x = 1
            pub.publish(command)
            return
        currentLaserTheta = currentLaserTheta + angleIncrement	

    print 'GoalAngle {}'.format(goalAngle)
    if abs(goalAngle - currentAngle) > 0.001 or goalDist > 0.001:
        command.linear.x = goalDist*0.1
        command.angular.z = (goalAngle - currentAngle)
        pub.publish(command)
        return

    # based on the motion you want (found using goal location,
    # current location, and obstacle info), set the robot
    # motion
    command.linear.x = 0
    
    
    pub.publish(command)

# main function call
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG)

    # subscribe to laser scan message
    sub = message_filters.Subscriber('base_scan', LaserScan)

    # subscribe to odometry message    
    sub2 = message_filters.Subscriber('odom', Odometry)

    # synchronize laser scan and odometry data
    ts = message_filters.TimeSynchronizer([sub, sub2], 10)
    ts.registerCallback(callback)

    # publish twist message
    pub = rospy.Publisher('cmd_vel',Twist)

    # Turn control over to ROS
    rospy.spin()



