import rospy import time import math
from geometry_msgs.msg import Twist # ROS Twist message
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# ROS topic names for turtlebot_gazebo
motionTopic = '/cmd_vel'
imageTopic = '/camera/rgb/image_raw'
poseTopic = '/odom'
safeTopic = '/safe_vel'
targetSideTopic = '/side_of_target'
# Global variables
gCurrentImage = Image() # make a global variable for the image gBridge = CvBridge() # make a ROS to CV bridge object
gLoc = [0, 0, 0] # x,y,yaw pose of robot
gTarget_side = 1
#
# Callback for the image topic #
def callbackImage(img):
    '''Called automatically for each new image'''
global gCurrentImage, gBridge
gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8") return
#
#
#
def sideCallback(data):
global gTarget_side gTarget_side = data.linear.x return
#
# poseCallback
  # this procedure is called to accept ROS pose topic info # and make it available via the global gLoc
#
def poseCallback(data):
global gLoc
gLoc[0] = data.pose.pose.position.x
gLoc[1] = data.pose.pose.position.y
orient = data.pose.pose.orientation
quat = [orient.x, orient.y, orient.z, orient.w] (roll, pitch, yaw) = euler_from_quaternion(quat) gLoc[2] = yaw
return
#
# procedure to track a colored region of the image by # rotating the robot so that the colored region remains # centered. The color information is a min and max
# RGB value in targetCol=[minrgb,maxrgb]
#
def colorNode(targetCol, start_time, color_num):
    '''center the robot camera on the target if in view'''
global gCurrentImage global gTarget_side
rospy.init_node('colorNode', anonymous=True)
# create windows to show camera and processing cv2.namedWindow('Turtlebot Camera') # , cv2.WINDOW_AUTOSIZE) cv2.namedWindow('Target') # , cv2.WINDOW_AUTOSIZE)
imageSub = rospy.Subscriber(imageTopic, Image, callbackImage) rospy.Subscriber(poseTopic, Odometry, poseCallback)
safe_pub = rospy.Publisher(safeTopic, Twist, queue_size=0) rospy.Subscriber(targetSideTopic, Twist, sideCallback) rospy.sleep(5) # wait for callbacks to catch up
    rate = rospy.Rate(10)
    msg = Twist()
    init_time = time.time()
    print("Looking for the next target ...")
while not rospy.is_shutdown():
# just show what the camera sees now
cv2.imshow('Turtlebot Camera', cv2.resize(gCurrentImage, (320, 240)))
        # get height h and width w of current image

  target
h, w = gCurrentImage.shape[0], gCurrentImage.shape[1]
# make a binary image that is 0 except where the color is in range
targetImage = cv2.inRange(gCurrentImage, targetCol[0], targetCol[1])
cv2.imshow('Target', cv2.resize(targetImage, (320, 240)))
# tracking algorithm
threshold = 1000000
if gTarget_side == 1: # 1 means target was on the right
avel = 0.7 # default velocity, so robot 'spins' when no target in view else: # if target was on the left
    avel = -0.7
lvel = 0.0
time_spinning = time.time() - init_time
if time_spinning >= (2*math.pi)/abs(avel): msg.angular.z = 0
msg.linear.x = 0.5 safe_pub.publish(msg)
    init_time = time.time()
    rospy.sleep(1)
# extract the moments of the target image
m = cv2.moments(targetImage)
if m['m00'] > threshold: # skip if the target image is not big enough
    # how far is the X center of target  from X center of image
delx = w/2 - m['m10']/m['m00']
avel = 0.4*delx/w # use this to generate a proportional ang velocity dist = m['m00']/(h*w) # area as a fraction of the image size
A, epi = 50, 10 # target area size, controls how close ti get to
if dist > A+epi: lvel = -0.1
elif dist < A-epi: lvel = 0.2
else:
elapsed = time.time()-start_time
txt1 = f"Goal: {color_num}"
txt2 = f"Loc: {int(gLoc[0])}, {int(gLoc[1])}"

  {txt3}\n\n")
txt3 = f"Tracking Time: {round(elapsed,2)}"
font = cv2.FONT_HERSHEY_DUPLEX
green = (0, 255, 0)
print(
    f"**** FOUND target number {color_num}\n**** {txt2}\n****
cv2.putText(gCurrentImage, txt1, (0, 50), font, 2, green)
cv2.putText(gCurrentImage, txt2, (0, 110), font, 2, green)
cv2.putText(gCurrentImage, txt3, (0, 170), font, 2, green)
filename = f"target{color_num}.jpg"
cv2.imwrite(filename, gCurrentImage)
msg.linear.x, msg.angular.z = lvel, avel # publish velocity safe_pub.publish(msg)
return
            print(
                f"Target Size = {round(dist,2)} => lvel = {round(lvel,2)}; delx =
{round(delx,2)} => avel = {round(avel,2)}")
msg.linear.x, msg.angular.z = lvel, avel # publish velocity safe_pub.publish(msg)
cv2.waitKey(1) # need to do this for CV2 windows to show up rate.sleep()
msg.angular.z = 0 msg.linear = 0 safe_pub.publish(msg) return
#
#
#
def callback_shutdown():
print("\nAll targets done! Give us an A please")
pub = rospy.Publisher(motionTopic, Twist, queue_size=1) msg = Twist()
msg.angular.z = 0.0
msg.linear.x = 0.0
pub.publish(msg)
rospy.sleep(5)
return
if __name__ == '__main__':

  try: rospy.on_shutdown(callback_shutdown) color_num = 0
start_time = time.time()
    targetColor = [[(0, 30, 75), (5, 50, 89)],
                   [(240, 240, 240), (255, 255, 255)],
[(0, 80, 80), (10, 110, 110)],
[(0, 0, 0), (20, 20, 20)]] # orange, white, yellow, black
for color in targetColor:
colorNode(color, start_time, color_num) color_num += 1
except rospy.ROSInterruptException: pass
