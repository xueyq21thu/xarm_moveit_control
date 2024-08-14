# Communication Interface

## Overview

1. VR Point cloud rendering, with fixed camera xyz position and floating orientation.
2. Orientation of Point Cloud can be held and dragged by the user.
3. The gripper can be dragged and moved by the user to set the desired position, which needs to send the position to ROS. (float32 [] in 6 dim)
4. The force vector can be designed by user and sent to ROS.(float32 [] in 6 dim). The force vector is attached on the gripper.
5. An action command can be sent to ROS to start the robot movement planning and execution. The robot will move to the desired position and apply the force vector.
6. After the command is sent, the robot will move to the desired position and apply the force vector.

## VR Interaction

- Fixed 
