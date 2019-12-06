# follow_the_gap_realsense


## How to start the ros nodes to run our car

1) Start the pololu node
```
roslaunch ros_pololu_servo pololu_example.launch
```
Code used for ros_pololu can be found at https://github.com/geni-lab/ros_pololu_servo
If you want to send a single command to the pololu run a command of the form:
```
rostopic pub -1 /pololu/command ros_pololu_servo/MotorCommand "joint_name: 'steering'
position: 1.0
speed: 1.0
acceleration: 0.0" 

```
In a separate window when the ros_pololu node is running.

2) Start the Realsense node
```
roslaunch realsense2_camera rs_camera.launch
```
Code used for the realsense node can be found at https://github.com/IntelRealSense/realsense-ros

