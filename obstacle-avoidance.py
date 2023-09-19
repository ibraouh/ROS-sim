import math
import rospy # needed for ROS
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# ROS Topics
#
motionTopic = '/cmd_vel' # turtlebot vel topic poseTopic = '/odom'
laserTopic = '/scan'
safeTopic = '/safe_vel'
targetSideTopic = '/side_of_target'
# global variable, to communicate with callbacks
gLoc = [0, 0, 0] # x,y,yaw pose of robot
gVel = [0, 0]
gBumperLeft, gBumperRight = False, False # left/right close gHeading = 0
# poseCallback
# this procedure is called to accept ROS pose topic info
# Ignore this - its just for the Gazebo simulation
#
#
#
def poseCallback(data):
global gLoc, pctr
gLoc[0] = data.pose.pose.position.x
gLoc[1] = data.pose.pose.position.y
orient = data.pose.pose.orientation
quat = [axis for axis in [orient.x, orient.y, orient.z, orient.w]] (roll, pitch, yaw) = euler_from_quaternion(quat)
gLoc[2] = yaw
return
# # #
# ROS Twist message
# ROS laser msg
# ROS odometry
  def safeCallback(data): global gVel
    gVel[0] = data.linear.x
    gVel[1] = data.angular.z
# callback for the laser range data
# calculates list of laser range contacts
# in world cartesian coordinates and maps them #
def callback_laser(msg):
    '''Call back function for laser range data'''
global gBumperLeft, gBumperRight, gVel
gBumperLeft, gBumperRight = False, False obstacle = False
numRays = len(msg.ranges) # total num readings radPerIndex = math.radians(360)/numRays
center = 0 # laser points to the front
width = int(numRays/4) # left/right bumper 'window' tooClose = .5 # threshold for bumper to activate
for i in range(0, len(msg.ranges)):
# rule out bad readings first
# if not math.isnan( msg.ranges[i] ) and \
# not math.isinf( msg.ranges[i] ) and msg.ranges[i]>0:
        # check for anything close left and right
if msg.ranges[i] < tooClose: if i in range(0, width):
gBumperLeft = True
elif i in range(numRays-width, numRays):
gBumperRight = True
return
# wander_node - version 1
# Moves forward until it detects a close surface
#
def oa_node(): # need make_map function included in node?
    '''continually move forward until a close surface is detected'''
global gBumperLeft, gBumperRight global gHeading
    # all ROS 'nodes' (ie your program) have to do the following
rospy.init_node('OA_Node', anonymous=True)

      # register as a ROS publisher for the velocity topic
vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
# register as a subscribe for the laser scan topic
scan_sub = rospy.Subscriber(laserTopic, LaserScan, callback_laser) # register as a subscriber for the pose topic rospy.Subscriber(poseTopic, Odometry, poseCallback)
    # register as a subscriber for motion information published by colorNode
    rospy.Subscriber(safeTopic, Twist, safeCallback)
    sideTarget = rospy.Publisher(targetSideTopic, Twist, queue_size=1)
# this is how frequently the loop below iterates rate = rospy.Rate(10) # Hz
msg = Twist() # new velocity message value = Twist()
msg.linear.x, msg.angular.z = 0, 0 vel_pub.publish(msg) # stop all motors
while not rospy.is_shutdown():
msg.linear.x, msg.angular.z = gVel[0], gVel[1] obstacle = 0
if gVel[0] != 0.0:
if gBumperLeft and gBumperRight: # go back
                gHeading = gLoc[2]
                msg.linear.x, msg.angular.z = -0.5, 0.5
                obstacle = 1
                print("In front")
elif gBumperLeft: # turn right
gHeading = gLoc[2]
msg.linear.x, msg.angular.z = gVel[0]/2, -0.2 obstacle = 1
print("Left")
elif gBumperRight: # turn left
gHeading = gLoc[2]
msg.linear.x, msg.angular.z = gVel[0]/2, 0.2 obstacle = 1
print("Right")
        # reorienting
if obstacle:
angleDiff = (gHeading - gLoc[2])
if angleDiff > 0: # then the robot is facing too far "right" and
should turn "left"
                value.linear.x = 1

sideTarget.publish(value)
elif angleDiff < 0: # then the robot is facing too far "left" and should turn "right
                value.linear.x = 0
                sideTarget.publish(value)
        vel_pub.publish(msg)
        rate.sleep()
return
#
# This function is called by ROS when you stop ROS # Here we use it to send a zero velocity to robot # in case it was moving when you stopped ROS
#
def callback_shutdown():
print("Shutting down")
vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=10) msg = Twist()
msg.angular.z = 0.0
msg.linear.x = 0.0
vel_pub.publish(msg)
return
# -------------------------------MAIN  program----------------------
if __name__ == '__main__': try:
        rospy.on_shutdown(callback_shutdown)
oa_node()
except rospy.ROSInterruptException:
pass
