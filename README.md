Yumi Manager Package
==========================

Requirements
-----------
* Kinect like RGB-D sensor
* fact_launch package
* pcl
* OpenCV >= 3.1
* [ROS CPP Utils package](https://github.com/hkaraoguz/ros_cpp_utils.git)


This package is created for high level interfacing with Yumi using different motion control and user input interfaces. The user can use the simple GUI shown in figure to perceive the workspace and perform simple actions such as pointing, picking and returning Yumi to home configuration.

The `yumi_manager` package is developed using inheritance to embed the basic functionality by default. Thus the user can modify the existing high-level control code easily according to his/her needs.

Running it with Yumi Pedestal Setup
-----------------
For using the node on the Yumi pedestal setup first make sure that the Yumi is running with either `moveit` or `skill` motion control interface.

Simply launch the associated launch file to run the whole high-level control pipeline:
```
roslaunch fact_launch perception_<motion_control_interface>.launch
```
where `<motion_control_interface>` is either `moveit` or `skill`.
If everything is correctly running, you should see a simple GUI with camera feed. If you press the `brush` icon on the top left (next to `disk` icon) an additional menu will be shown.

### Additional GUI Menu
`Pick Place` and `Point` actions: These radio buttons are used for selecting the desired action. <br/>
`Home Position`: Returns the arms to home position.<br/>
`Plan Action`: If `moveit` is selected as the motion control interface, then this button is used to activate/deactivate the generation of motion plans for the desired action without executing. The generated plan is published under `/yumi_manager/moveit_trajectory` topic. <br/>
`Refresh Scene`: Refreshes the detected color blobs. <br/>
`Show Workspace`: If read correctly, this buttons shows/hides the current workspace limits on the camera view.




### Leap Motion Interface
Basic Leap Motion interface also exists for controlling the robot. In order to run this interface, first plug-in the sensor to the computer.

First run:
```
sudo service leapd restart
```
Then run the Leap Interface in a terminal:
```
LeapControlPanel
```
After running this command, you should see a small green bar on top left of the screen.

Run the leap manager:
```
roslaunch fact_launch perception_leap_<motion_control_interface>.launch
```
If everything works, you should see a pink dot in the camera view. This dot will move on the screen based on the user's left hand movement. If the user closes his/her hand on top of a detected color segment, the selected action will be executed for that segment.
