# ROS-sim
Simulator for ROS to control the Turtlebot Gazebo robot so that it uses visual navigation and obstacle avoidance to search and go to each of the four colored targets that are in our gazebo simulation


## 1.0 Objective
The purpose of this Lab is to construct a ROS (Robot Operating System) program that will control the Turtlebot Gazebo robot so that it uses visual navigation and obstacle avoidance to search and go to each of the four colored targets that are in our gazebo simulation.

**Figure 1:** Gazebo Enclosure with 3 colored walls at each corner
This simulation includes a different color at each edge of the wall enclosure (black, white, yellow, orange). The simulation also includes 4 columns placed in the center of the enclosure to represent the obstacles for the robot.

### 1.1 Task
In this Lab, the robot will start in the center between all 4 columns and navigate using visual sensing to each of the colored sections of the walls while avoiding obstacles on its way using an obstacle avoidance program. Once the robot gets close enough to each colored wall, the robot will take a picture from its camera and write its current position, current goal number, and time elapsed on the picture as shown in the figure below.
 
## 1.2 Different Parts of the Lab
This project is divided into two main parts:
- Part 1 solves the visual sensing problem and implements a program that would lead the robot to each of the different color targets as well as take pictures of each of the colored walls while taking into account the current position, and the time elapsed.
- Part 2 will implement an obstacle avoidance program that will simply vary the angular velocity of the robot when it senses an obstacle in the robot’s path using its sensors.
This lab report will be laid out as follows: Section 2 will discuss the system design of the color tracking node, while section 3 will go over the implementation of this visual tracking program. In Section 4, we will go over the design and implementation of the obstacle avoidance node. And finally, Section 5 will go over merging both parts to work together as well as the testing and tuning of both nodes that was required.

## 2.0 System Design of the Color Tracking Node
The Color Tracking Node’s goal is to go through each of the four target colors, and look for walls that match that color within the enclosure. An obvious way to do this is to first locate the target, orient towards that target, and finally move towards it. We also need to reacquire the target when it’s lost during the movement, as well as figure out what the next steps should be if the target could not be found from the robot’s initial position. Finally, and most likely the most important part, is that we need to establish a way of communication between the Obstacle Avoidance Node and the Color Tracking Node. These sections will go over the design and will be split into different steps that were essential to the development of this node.
**Figure 2:** Example of the returned picture

### 2.1 Locating the target
To locate a target, the robot starts at any location and rotates around its axis. During the rotation, the robot uses the computer vision package to scan using its camera and its surroundings and looks for the specific color that was passed in its iteration from the frames in the picture returned by the camera.
An important part in this quest is to specify what sizes of the specific color the robot should consider in the enclosure. Since this enclosure is not perfect and there are a lot of spots that may resemble the colors passed, especially black and white, the robot needs to figure out which one is too small to pursue and which one may be the actual target (See more in Section 3).
We do the following within the main while loop inside the colorNode function as follows:
```
 While the robot is not stopped
Get image from computer vision
Scan the image for any spots that are the same as the color we
are looking for
Return an image that is completely black except for the spots
that are the same color we are looking for Rotate the robot with a set angular velocity of 0.5 Look for spots large enough to be targets
Center these spots in the picture frame
Stop the robot
Return
```
**Figure 3:** Pseudo-code for the target location
### 2.2 Requiring a lost target
When completing the rotation and finding the target, the robot’s vision of the target could get obscured by an obstacle. To combat this problem the robot has been programmed to stop, complete another full rotation to check if the target is to its sides or behind it. If the robot couldn’t find the target, the linear velocity is increased to move the robot away from the obstacle. Once the robot moves, it completes another spin to locate the target again.
### 2.3 Moving towards a target
Once the robot finds a target, it moves directly towards it while keeping the center of the target returned from the camara’s readings through computer vision in the middle of the frame. This can be done using the proportional control law where the angular correction can be decided on

 the function of the target’s occupation on the camera. If the target’s center is not aligned by the center of both axes of the camera, the program automatically corrects that.
Moreover, we can decide the linear speed of the robot in function of the area of the color wall on the screen. A value for the area that works in this situation is 50. We can either increase the linear speed if the target’s area is smaller than 50, or reverse it to backup if the target’s area is larger than 50.
### 2.4 Taking a picture and logging data
When the robot reaches a satisfactory distance from the target, it takes a picture of that target while including some information about the target and the robot in the picture. This information includes the number of the goal that ranges from 0 to 3 that is passed in the main function call, the current coordinates of the robot taken from the pose topic, as well as the elapsed time since the program started running using the time python package.

## 3.0 Implementation of Color Tracking Node Please refer to Appendix A for this section of the lab. The implementation of this program was as follows:
 While robot is not shut down:
   Get image from computer vision
   Scan the image for any spots that are the same as the color we are
looking for
   Return an image that is completely black except for the spots that
are the same color we are looking for
   Rotate the robot with a set angular velocity of 0.5
   Look for spots large enough to be targets
   Center these spots in the picture frame
   Stop the robot
   If robot can't find spots after a full rotation:
       Move the robot forward for 1 second at a speed of 0.5 m/s
   Move toward the goal while keeping it center
   When the robot is close enough:
       Stop robot
       Log location and tracking time
       Save image with the information
Return

**Figure 4:** Pseudocode for the Color Tracking main while loop
A very important part of the implementation is the communication between the Color Tracking Node (ColorNode) and the Obstacle Avoidance Node (OANode). A new topic called /safe_vel was established and implemented to be a one way communication between the nodes, where ColorNode publishes the linear and angular velocities as Twist to the topic, and the OA node subscribes to this topic and reads those Twists to then publish the to the /cmd_vel topic to move the robot. This way we can avoid contradicting velocities being published to the /cmd_vel topic if both nodes were to publish directly.
When the ColorNode ran a total of 4 times, the following elapsed time readings were recorded:
**Figure 5:** Running time (in seconds) per obstacle
As it can be seen from the figure, the running time for color 1 and color 2 were very similar in all our tests that started at different locations, however, when we get to color 3 and 4, the divergence between the test can be seen. It takes considerably longer for the robot to locate the third color (white) and picks up many spots on the map that are the same color. In fact, the program failed multiple times trying to locate the white and black walls when chasing spots on the enclosure that resembled those colors.
To fix this, the threshold value for the computer vision readings from the moments value had to be changed as follows:
  m = cv2.moments(targetImage)
if m['m00'] > threshold: # skip if the target image is not big enough

 The target is only to be followed after the threshold value is met. We initially started with threshold being 0, which was too low as it picked up even the slightest spots in the computer vision. Through experimentation, when the value of the threshold was increased to 1,000,000, the accuracy became much better and the if statement only kicked in when the target was large enough.
When changing these results and running the test again we received the best results yet with a running time of 149 seconds as well as the following partitions of each color detection:
**Figure 6:** Partition of Running time for each color
The Color Detection Node was completed and ready to be tested with the Obstacle Avoidance Node.

## 4.0 Design and Implementation of the Obstacle Avoidance Node
Please refer to Appendix B for this section of the lab.
The Obstacle Avoidance was fairly straightforward. It uses obstacle avoidance using the laser readings of the robot then makes the robot either rotate to the left, right, or back up from the obstacle. The program’s while loop is laid out as follows:
  If robot is moving:
Velocities -> Velocities given by CTNode If Obstacle on the right and left:
Set linear velocity to -0.5 (back up) Set angular velocity to -0.1 or 0.1
         Reorient to initial angle
```
       Else If Obstacle to the left:
        Set angular velocity to -0.2 (Turn to the right)
        Set linear velocity to current_velocity/2
         Reorient to initial angle
     Else If Obstacle to the right:
        Set angular velocity to 0.2 (Turn to the left)
        Set linear velocity to current_velocity/2
         Reorient to initial angle
      Publish the velocities
```
**Figure 7:** Pseudocode for the Obstacle Avoidance Node
From the Pseudocode, we can see that the Obstacle Avoidance only kicks in when the robot is
moving, and passes all the necessary velocities to avoid the obstacles on the way.
It is also important to note that the Obstacle Avoidance Node is the only one that controls the velocities of the robot. The Obstacle Avoidance Node receives the velocities that the Color Tracking Node wants, and then checks for obstacles. If any obstacles are detected, the velocities are changed in order to avoid the obstacle. Otherwise, if no obstacles are detected, the velocities are published to the robot as planned.

## 5.0 Tuning and Testing both Nodes
When testing the Color Tracking Node alone, no specific issues were detected, however when it was executed along with the Obstacle Avoidance Node, there were some issues to take care of.
The first test failed multiple times as the robot either got stuck at obstacles, or infinitely looked for the target without finding it. To solve this problem we’ve created another channel of communication between both nodes through a new topic caller /side_of_target. We use this topic in order to reorient the robot after each obstacle avoidance and it is divided into two tasks for each node.
In the initial run, the robot would be heading toward a target as planned, however when it encountered an obstacle it would start the obstacle avoidance, which means if the obstacle were to the right, the robot would rotate to the left, and the inverse for the left obstacles. This would also mean that the robot’s field of view must not have the target anymore, as it’s not in the same angle that it initially was before the obstacle avoidance started.

 The solution here started by taking note of the robot’s yaw angle before it started obstacle avoidance, as well as after it stopped. This way, by subtracting the initial angle from the current angle, we could figure out if the robot derived to the right, or to the left. This would also help us conclude if the target is on the right (by returning 1 to the topic) or left (returning 0) side of the robot.
For the second part of the solution, we take those binary values that tell the side in which the target is in relation to the robot and we reorient it towards it by varying the default angular velocity from -0.7 to 0.7.
After making this change we intensively tested the program by running it from each corner as well as on the 5 main positions between the obstacles and we got the following results:
**Figure 8:** Running time per Starting Position
From the figure above, we got some really good results, and the best run averaged 167.8s much
faster than when we first started.

## 6.0 Conclusion
In conclusion, a robot operating system program was made to make a turtlebot3 robot navigate an enclosure with obstacle avoidance, as well as tracking 4 specific color targets within this enclosure and logging some information about those targets in respective pictures taken by the robot’s camera. This was done using two nodes: Color Tracking Node, which helped the robot

navigate to each of the targets and take their pictures, and the Obstacle Avoidance Node, which helped avoid the cylinder obstacles along the way using the robot’s sensors and bumper algorithm.

Initially, we wanted to use the potential fields map in order to have a smoother navigation of the enclosure, however, we could not get the map to change the behavior of the robot and decided to implement it using the laser readings. With more time, we would have finished the implementation of the potential fields map which would have most likely led to a faster runtime.
