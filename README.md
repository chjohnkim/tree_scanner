# Instructions for using the tree scanner
1. Startup terminator and load custom layout
```
terminator -l john
```
2. Sequentially execute terminator windows from left right top down order. This will execute the following commands FYI:
```
roscore
# Gives permission to usb devices, sets buffer size and launches syncbox_control node
sh init1_syncbox_control.sh
# Publishes some topics
sh init2_rostopic_pub.sh
# Linear slider client
roslaunch servo_controller servo_client.launch
# Linear slider server
roslaunch servo_controller servo_server.launch
# Connect to robot
roslaunch ur_modern_driver ur5_bringup_joint_limited.launch robot_ip:=192.168.1.10
# Moveit planner
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
# RVIZ
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
# Calibrate slider
rostopic pub /do_calib std_msgs/String "t"
# Launch camera driver
roslaunch spinnaker_camera_driver firefly_color_launch.launch
# rqt for image visualization
rosrun rqt_gui rqt_gui
# Launch apriltags_ros node
roslaunch apriltags_ros john.launch
```

3. After calibrating the linear slider, you need to put the slider back to its original position. Use RVIZ motion planning interface for that:
- Under **MotionPlanning** section, go to **Planning** tab
- Under **Query**, choose **Select Start State** and **Update** start state to **\<current\>**
- Click **Plan**, make sure it goes to the desired position in the visualization, and click **Execute**

4. Open a new terminator window next to RVIZ and execute main.py in the script folder, which executes the scanning routine.
```
python main.py
```
- Follow instructions in terminal. 
- **WARNING:** Make sure to always check that the robot is able to follow 1.0 of the motion plan, and the visualization in RVIZ looks correct.

# Hyperparameters in main.py:
```
MODE = 'sphere' # Make sure mode is set to sphere
ROBOT_VELOCITY = 0.3 # Max robot speed scaled from 0 to 1
SCAN_RADIUS = 0.3 # Radius of sphere
SPHERE_LEVELS = [0.90] # Scanning angle on sphere
NUM_VIEWPOINTS = 16 # Number of images to take excluding start and end images
```











