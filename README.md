# ele306_semesteroppgave
This is a school project from HVL.

## Felles mappe
https://hvl365-my.sharepoint.com/:f:/g/personal/583663_stud_hvl_no/EkC8XdJOopJJqDhuiiCXXMwBQ5gGoz-AVumh3nnX7u4V-A?e=sfdB2A

# Robot Design Challenge 3: A couple of kitchen arms to prepare basic food
## Robot arm requirements:
a. Two 6-dof robots arms attached on the wall,

b. Both arms are able to reach the ingredients in their
workspace. All necessary ingredients are placed into
workspace by a human.

c. As grippers there are 3-finger, smasher, paddle and cutter
grippers which are placed in the workspace and the robot
arms can replace them by themselves
## Mobile base requirements:
a. Both arms must be able to reach all the points of a
70*150cm kitchen bench
## Sensor requirements:
a. There must be a smoke sensor in case of emergency

b. There must be one camera to distinguish ingredients/ tools
etc.

c. There must be a timer to inform user about the process

d. One user input tablet can be used for basic menu input


○ Expected result: An omelet with 3 ingredients.

## ¤
# Project requirements
Most of the project design challenges are focused on a mobile manipulator. That is, a robotic system
with robot arm(s), a mobile robot platform, and a sensory system. Each project will be slightly
different, as the challenges are quite diverse, and as each group may choose a different way to
approach a given robot design challenge. However, all projects must include the following, which
should be documented in the report, or in appendices to the report:

## 1. Develop the forward kinematics of your robotic solution, in Matlab (not Toolbox) or by hand:
### i. For the robot arm(s):
1. Develop the table of DH parameters

2. Develop the transformation mapping end-effector to base (for the first 4
joints only)
### ii. For the mobile robot platform(s):
1. Draw a model of the mobile robot with the necessary variables defined
(see Fig. 4.1 in Corke for inspiration)

2. Develop the kinematic equations of motion for the mobile robot

3. Discuss whether the mobile robot is holonomic or non-holonomic
### iii. For the robotic system in general:
1. Develop the transformation from the chosen sensor system to the
relevant coordinate system on the robot (world, end-effector, mobile
robot, etc)
## 2. Model your robot kinematics with Peter Corke's Robotics Toolbox in Matlab:
### i. For the robot arm(s):
1. Demonstrate equivalence of the forward kinematic solution obtained
previously in Matlab (not Toolbox) or by hand

2. Develop the differential kinematics (i.e. relating joint and Cartesian
velocities), and demonstrate how it could be used

3. Develop the inverse kinematics, and demonstrate how it could be used

4. Demonstrate example motion planning, on a task relevant to your robot
design challenge (or similar)
### ii. For the mobile robot platform(s):
1. Determine suitable controller(s) to control the mobile robot for your
chosen challenge

2. Implement the kinematic model and the controller(s) in Matlab
(/Simulink)
### iii. For the robotic system in general:
1. Demonstrate using the sensory system to command the robot,
according to the task chosen. That is, show the calculations necessary to
make the sensory data (e.g. an apple detected at an arbitrary location
from a static 3D camera) useful to the robot (e.g. calculate the joint
angles to put the tool point of the end-effector at the apple’s location).
## 3. Simulate the kinematics of your robot in Matlab:
### i. For the robot arm(s), depending on robot design challenge either:
1. Use motion planning to move the robot end-effector through the
required positions/orientations for the task chosen, or

2. Use differential kinematics to move the end-effector using velocity
commands according to the task chosen
### ii. For the mobile robot platform(s):
1. Simulate your chosen challenge, and discuss the simulation results in
terms of chosen control strategy and performance

2. Discuss and implement a navigation strategy for the mobile robot for
your challenge

3. Discuss how you would implement a localization strategy for the mobile
robot for your challenge
## 4. Connect the Matlab code to ROS and simulate the physical robot in Gazebo
### i. For the complete system: Model your complete robot system using URDF and
visualize the robot in Gazebo, including:
1. Your robot arm(s) mounted on your mobile platform

2. Your mobile platform, with wheels, sensors etc
### ii. For the robot arm(s): Demonstrate controlling your robot arm(s) in Gazebo over ROS from Matlab, by following along a trajectory calculated in Matlab, or controlled using your differential kinematics implemented in Matlab.
### iii. For the mobile robot(s): Demonstrate controlling your mobile robot platform in Gazebo over ROS from Matlab.
## 5. Optional, if relevant: Control a physical UR, Turtlebot, or other robot using coordinates calculated with your Matlab code through ROS.
