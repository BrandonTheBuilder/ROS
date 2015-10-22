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
    # vx = command.linear.x
    # az = command.angular.z
    # import IPython; IPython.embed()
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
    if goalX != currentX and goalY != currentY:

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
        rVector = [0,0]
        for curScan in range(0, numScans):
            if distanceArray[curScan] != 30:
                d = distanceArray[curScan] - 1 
                # print 'd = {}, theta = {}, x = {}, y = {}'.format(
                #     d, currentLaserTheta, d*math.cos(currentLaserTheta), d * math.sin(currentLaserTheta))
                x = d*math.cos(currentLaserTheta)
                y = d*math.sin(currentLaserTheta)
                rVector[0] += -x
                rVector[1] += -y
                currentLaserTheta = currentLaserTheta + angleIncrement

        C = 1
        P = 3
    	goalVector = [(goalX - currentX) * math.cos(currentAngle) 
                    - (goalY - currentY) * math.sin(currentAngle),
                    (goalX - currentX) * math.sin(currentAngle) 
                    + (goalY - currentY) * math.cos(currentAngle)]
        print "goalVector {}".format(goalVector)
        print "Rvector {}".format(rVector)
        
        rVector[0] += goalVector[0] * 10
        rVector[1] += goalVector[1] * 10
        rVector = [C/math.pow(rVector[0],3), C/math.pow(rVector[1],3)]
        

        # based on the motion you want (found using goal location,
        # current location, and obstacle info), set the robot
        # motion
        m = math.pow(math.pow(rVector[0],2) + math.pow(rVector[1],2), 0.5)
        theta = math.atan2(rVector[1], rVector[0])
        print(theta)
        
        command.linear.x = m/.01
        command.angular.z = theta/0.1
        pub.publish(command)
    else:
        command.linear.x = 0;
        command.angular.z = 0;

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

