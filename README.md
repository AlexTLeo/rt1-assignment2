**Institution:** Università di Genova<br>
**Course:** MSc in Robotics Engineering<br>
**Subject:** Research Track 1<br>
**Author:** ***Alex Thanaphon Leonardi***<br>

# Assignment 2

## Introduction
This is the second assignment of the "Research Track 1" course, for the Robotics Engineering degree, Università di Genova.
It is a controller that runs on ROS and use the stageros node, which is a simulator in which there is a robot in a race track. The robot must autonomously complete the track whilst avoiding the track walls.

## Running the program
The simulator requires ROS. Clone the repository into your ROS workspace's src/ folder. Then, back in the workspace, run:
```
catkin_make
```
Then, you have two options:
### Option 1
First, run the stageros node
```
rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
Finally, run the controller and ui node
```
roslaunch second_assignment_controller second_assignment_controller_node
rosrun second_assignment_controller second_assignment_controller_ui_node
```
### Option 2: one-liner
Use the roslaunch file
```
roslaunch second_assignment_controller controller_ui.launch
```

## Description
The **controller node** works with the **stageros node**, which should also be running. The communication with the aforementioned node occurs through **topics**, to which the controller subscribes. In particular:
1 - "/base_scan" from which sensor data is acquired (related to obstacles)
2 - "/cmd_vel" through which the robot can be commanded with ROS geometry_msgs::Twist messages

The controller also uses **services** to communicate both with the **stageros node** and the **ui node** (more on this below). In particular:
1 - "/reset_positions" tells stageros to reset the robot to its starting position with an std_srvs::Empty message
2 - "/controller_reset" listens to the **ui node** for a reset command in the form of an std_srvs::Empty message
3 - "/controller_vel" listens to the **ui node** for a velocity increase/decrease command in the form of an std_srvs::SetBool command, where TRUE means "increase velocity" and FALSE means "decrease velocity" (specifically, the velocity is multiplied or divided by 2)

The **ui node** waits for user input and forwards certain commands to the **controller node** through the use of the services previously described. The commands are:
1 - "r" - reset robot position
2 - "w" - increase robot velocity
3 - "s" - decrease robot velocity

### Extras
The **roslaunch** file is set by default to print to the terminal. The program is capable of **logging** and can be set to do so, to the default ROS logs, by commenting and uncommenting the appropriate tags in the **launch/controller_ui.launch** file.

## Pseudocode
### controller node
```
while simulating
  divide scanner into 4 sectors
  check scanner sectors for obstacles

  listen for ui node

  if ui node sends command
    if command == reset
      reset robot position
    else if command == speed
      change robot base speed

  if obstacle close
    speed = proportional to obstacle distance and to base speed (given by ui node)

    if obstacle in left sector or left-front sector
      turn right
    else
      turn left
  else
    drive straight
```

### user interface (ui) node
```
while simulating
  listen for user input

  if key pressed == r
    send reset command to controller node
  elseif key pressed == w
    send velocity increase command to controller node
  elseif key pressed == s
    send velocity decrease command to controller node
```
