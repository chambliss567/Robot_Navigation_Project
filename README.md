## Overview 

This project was completed on the Construct platform, an online learning platform with dozens of courses teaching different fundamentals of ROS and robotics

This project sets out to fulfill the requirements defined in the **Navigate with a Real Robot** ROS project. The project specifications and requirements are as follows

### Specifications: 
1. A turtlebot3 will be used with a TF tree defined in **frames.pdf**
2. This turtlebot is placed within a track enclosed by 4 walls
3. This track contains multiple obstacles
4. Laser scan data is published to the /scan topic
5. Odometry data is published to the /odom topic

### Requirements
1. The robot will create a map of its environment using the gmap package
2. The robot will save and load this map using the map_server package
3. The robot will localize using the amcl node
4. A custom service will save the robot's current position to a text file when called
5. Using the move_base node, the robot will plan its path to a specific destination with a navfn global planner and a DWA local planner
6. A custom service will load specified destination coordinates from a text file and send those coordinates to the move_base node

[The video associated with this project](video_turtlebot_navigation.mp4) shows the robot navigating the environment in Gazebo
and in Rviz. For more details on the project and the rest of my ROS projects, visit my [profile at the construct sim](https://app.theconstructsim.com/#/Profile/mchambliss) and select **Rosjects**

## ROS Project Details

### Mapping

The **my_turlebot_mapping** package handles the turtlebot's mapping of its environment. **gmap.launch**
starts the slam_gmapping node with custom parameters. Once the robot has mapped its environment, the map
is saved in the maps directory. This map is loaded via **load_map_file.launch**

### Localization

The **my_turtlebot_localization** package handles the turtlebot's localization. **launch_amcl.launch**
starts **load_map_file.launch** from **my_turtlebot_mapping**. starts Gazebo and TF transform publishers via **main.launch** from the **realrobotlab** package, and starts the amcl node with custom parameters
that can be found in the **amcl_params.yaml** file. 500-2000 particles were used for localization.
In addition this package contains the custom **/save_spot** service, which can be started with the **spot_recorder_service_server.py** file and uses custom service message **MyServiceMessage.srv**. This service takes a string
argument *label* as a request, saves the robot's current amcl_pose under *label* in **spots.txt**, and returns a boolean and string result message indicating if the service was succesful.

### Path Planning

The **my_turtlebot_path_planning** package handles both the robot's global and local path planning. **move_base.launch**
starts **launch_amcl.launch** from **my_turtlebot_localization** and starts the move_base node with custom parameters that can be found in a number of yaml files found in the config directory. These parameters are modified from the path planning configuration found in the **ros-noetic-turtlebot3-navigation** package to best suit the environment for this project.
The DWA local planner is used for planning with the local costmap and the navfn global planner is used for global costmap path planning.

### Navigation

The **my_turtlebot_navigation** package contains a custom service **/move_to_spot** which can be started with the **move_to_destination_service_server.py** and **launch_move_to_destination_service.launch** files. This service uses the **MyServiceMessage.srv** message defined in the **my_turtlebot_localization** project. The service will load, from **spots.txt** in the **my_turblebot_localization project**, the pose associated with the requested *label*, send that pose to the move_base node, and return whether or not the robot could succesfully reach the desired position. Additionally, this package contains **robot_navigation.launch**, which will start **move_base.launch** from the **my_turtlebot_path_planning** project and **launch_move_to_destination_service.launch**.

## Launching Node

Before starting this package, verify that the package is built and source with catkin

To start the needed services for navigation, run the following command:</br>
**roslaunch my_turtlebot_navigation robot_navigation.launch**

To start rviz with the desired configuration, run the following command:</br>
**rosrun rviz rviz -d nav_project_rviz_config.rviz**

To send a service request to the /move_to_spot service, run the following command:</br>
**rosservice call /move_to_spot *label***