Sensors & Control Group Project 
======

Setup guide
------

Welcome to our group project for Sensors and Control Autumn 2022. The following is a complete guide to setup and run the simulation. 


1. Install Gazebo (with Fetch) and Turtlebot packages. 

2. Pull all files from the github into your catkin workspace.  

3. Copy the two model files used from the "models" folder and paste them into your ".gazebo" folder from the two lines. This step is only to have the environment models in the simulation and is not necessary.
  
  `cp -r catkin_ws/src/models/maze .gazebo/models/` <br />
  `cp -r catkin_ws/src/models/table_marble .gazebo/models/`

4. Edit your bashrc file using `gedit ~/.bashrc` in the terminal and insert the following lines of code at the bottom. 

`export TURTLEBOT3_MODEL=waffle` <br />
`export ROS_MASTER_URI=http://localhost:11311` <br />
`export ROS_HOSTNAME=localhost` <br />

5. Run `source ~/.bashrc` in your terminal then build your catkin workspace using `catkin_make`.

6. Install the Turtlebot keyboard controller using 

7. Run the following four lines of code in your terminal. 

* `roslaunch fetch_gazebo fetch.launch` opens the Gazebo simulation along with Fetch robot, Turtlebot guider robot and pre-made world file.

![a1](./catkin_ws/src/pic/a1.png)

* `roslaunch visp_auto_tracker tracklive_usb.launch` opens the Fetch's RGB camera and Vision ViSP ROS package to track and obtain the pose from QR code.

![a2](./catkin_ws/src/pic/a2.png)

* `rosrun controller controller_node` runs the ROS node which initiates the path following algorithm for the Fetch robot.

* `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` initiates keyboard controls for the Turtlebot guider robot.
