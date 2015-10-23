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
import Util

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry

# the velocity command message
from geometry_msgs.msg import Twist

# instantiate global variables "globalOdom"
globalOdom = Odometry()

# constants defined for attraction and repulsion
ROBOT_CHARGE = -1
OBSTACLE_CHARGE = -1
GOAL_CHARGE = 1000
DT = 0.01
vx = 0.0
vy = 0.0
u = 0.0
omega = 0.0
theta = 0.0
spinning = False
moving = False
Pd = [0,0]
Ld = [0,0]
# method to control the robot
def callback(scan,odom):
    # the odometry parameter should be global
    global globalOdom
    global vx, spinning, moving, theta, Pd, Ld
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
        if not spinning and not moving:
        # for each laser scan
            fResult = [0,0]
            for curScan in range(0, numScans):
                d = distanceArray[curScan]
                if d != 30:
                    r = Util.cartFromPolar(d, currentLaserTheta)
                    globR = Util.rotZTheta(r, -currentAngle)
                    f = Util.coulombForce(globR, 
                        OBSTACLE_CHARGE * Util.scanAngleMod(currentLaserTheta), 
                        ROBOT_CHARGE)
                    fResult[0] += f[0]
                    fResult[1] += f[1]
                currentLaserTheta += angleIncrement

                

        	goal = [goalX-currentX , goalY-currentY]
            mGoal = math.pow(math.pow(goal[0],2) + math.pow(goal[1],2),0.5)
            unitGoal = [goal[0]/mGoal, goal[1]/mGoal]
            goalForce = Util.coulombForce(goal, GOAL_CHARGE, ROBOT_CHARGE)
            fResult[0] += goalForce[0]
            fResult[1] += goalForce[1]
            localF = Util.rotZTheta(fResult, currentAngle)
            m, theta = Util.polarFromCart(localF)
            

            print 'Force {}'.format(fResult)
            print 'U {}'.format(command.linear.x)
            global vx, vy, u, omega
            vx += fResult[0]*DT
            vy += fResult[1]*DT
            print "location: ({}, {})".format(currentX, currentY)
            print "Angle: {}".format(currentAngle)
            print "Goal: ({}, {})".format(goalX, goalY)
            print "GoalVector: {}".format(goal)
            print "ax: {}, ay: {}".format(fResult[0], fResult[1])
            print "vx: {}, vy: {}".format(vx, vy)
            Pd = [vx*DT, vy*DT]
            Ld = [currentX+Pd[0], currentY+Pd[1]]
            print "Desired Point: {} ".format(Pd)

            u, omega = Util.unicycleTracking(Pd, u, omega, currentAngle)
            if abs(currentAngle - theta) > .1:
                command.angular.z = omega
                spinning = True
            else:
                command.linear.x = u
            
        else:
            u, omega = Util.unicycleTracking(Pd, u, omega, currentAngle)
            if abs(currentAngle - theta) > .1:
                command.angular.z = omega/abs(omega)
                spinning = True
            elif abs(currentX - Ld[0]) < 0.1 and abs(currentY - Ld[1])< 0.1:
                print 'u: {}'.format(u)
                print 'location: ({}, {})'.format(currentX, currentY)
                command.linear.x = u/abs(u)
                spinning = False
                moving = True
            else:
                spinning = False
                moving = False
    else:
        pass
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

