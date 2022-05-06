# SNCFETCH
Sensors &amp; Control group project for Fetch robot following the path of a guider robot. AUT2022.

Welcome to our group project for Sensors and Control Autumn 2022. The following is a complete guide to setup and run the simulation. 

1. Install Gazebo (with Fetch) and Turtlebot packages. 

2. Pull all files from the github into your catkin workspace.  

3. Copy the two model files used from the "models" folder and paste them into your ".gazebo" folder from the two lines. 
  
  `cp -r catkin_ws/src/models/maze .gazebo/models/`
  `cp -r catkin_ws/src/models/table_marble .gazebo/models/`

4. Build your workspace and run the following four lines of code in your terminal. 

roslaunch fetch_gazebo fetch.launch (Opens Gazebo simulation along with Fetch robot, Turtlebot guider robot and pre-made world file).
[INSERT IMAGE OF WORLD]

roslaunch visp_auto_tracker tracklive_usb.launch (Opens the Fetch's RGB camera and Vision ViSP ROS package to track and obtain pose from QR code).

[INSERT IMAGE OF QR CODE TRACKING]

rosrun controller controller_node (Runs ROS node which initiates the path following algorithm for the Fetch robot).

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch (Initiates keyboard controls for the Turtlebot guider robot).
