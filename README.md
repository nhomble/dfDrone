dfDrone
=======

detect and follow drone

## Nodes
- (camera)
- startDrone <--> detectDrone
- centerDrone
- moveDrone
- (cmd_vel_mux)

## Messages
- DFDMessage
- but these are really just done with Float64MultiArrays

## Prior to running, you must:
- $ roscore
- $ roslaunch turtlebot_bringup minimal.launch
- $ roslaunch openni_launch openni.launch
