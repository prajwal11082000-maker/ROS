# install ros2 humble on ubuntu:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


# install Git on Ubuntu
### sudo apt update
### sudo apt install git -y
### git --version
### git config --global user.name "Your Name"
### git config --global user.email "your_email@example.com"
### git config --list


# Step-by-Step: Add “Open With SmartGit” Option:

### Make sure your .desktop entry exists
### nano ~/.local/share/applications/smartgit.desktop

### Paste (or update) this content:
[Desktop Entry]
Name=SmartGit
Comment=Git client for developers
Exec=/home/enmac/Downloads/smartgit/bin/smartgit.sh %F
Icon=/home/enmac/Downloads/smartgit/bin/smartgit.png
Type=Application
Categories=Development;IDE;
Terminal=false
MimeType=inode/directory;


### Make it executable
### chmod +x ~/.local/share/applications/smartgit.desktop


### Update desktop database
### update-desktop-database ~/.local/share/applications

### Restart the file manager (optional)
### nautilus -q

# Nodes

https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html

# Topics

https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html

https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html

# Services

https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html

# Actions

https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

# Parameters

https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

# How to Run and Build

### Project Explanation

This project contains a ROS 2 Python package named my_first_pkg. It demonstrates a basic communication system with "talker" and "listener" nodes.

### Key Components:

Package Name: my_first_pkg

Type: ament_python (Python-based ROS 2 package)

### Nodes:

talker: Publishes messages (likely a "Hello World" counter).

listener: Subscribes to the messages published by the talker.

Launch File: talker_listener.launch.py (Launches both talker and listener simultaneously).


cd /home/enmac/ROS/ROS/ros2_ws

colcon build --packages-select my_first_pkg

source install/setup.bash

ros2 run my_first_pkg talker

ros2 run my_first_pkg listener

ros2 launch my_first_pkg talker_listener.launch.py


# Turtle Sim:

ros2 run turtlesim turtlesim_node

ros2 run turtlesim turtle_teleop_key

ros2 node list

ros2 node info /turtlesim

ros2 topic list

ros2 topic echo /turtle1/cmd_vel

ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

# URDF

ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/05-visual.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf


# run robot

enmac@enmac-OptiPlex-5060:~/proj_1$ colcon build --symlink-install

enmac@enmac-OptiPlex-5060:~/proj_1$ source install/setup.bash

enmac@enmac-OptiPlex-5060:~/proj_1$ bash -c "source install/setup.bash && ros2 launch my_robot_controller robot.launch.py"

# TF frames define where every part of the robot and world is located → and ROS 2 uses them to convert coordinates between sensors, robot, and map.

# nav2 generate map

enmac@enmac-OptiPlex-5060:~$ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*

enmac@enmac-OptiPlex-5060:~$ sudo apt install ros-humble-slam-toolbox

enmac@enmac-OptiPlex-5060:~$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

enmac@enmac-OptiPlex-5060:~$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

enmac@enmac-OptiPlex-5060:~$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

enmac@enmac-OptiPlex-5060:~$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

enmac@enmac-OptiPlex-5060:~$ ros2 run nav2_map_server map_saver_cli -f my_map
