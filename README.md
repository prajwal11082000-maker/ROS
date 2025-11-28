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

colcon build --packages-select my_first_pkg
call install\setup.bat

ros2 run my_first_pkg run_both

python -m pip install pyinstaller

pyinstaller --onefile --noconsole --name talker_listener d:\ROS\ROS\ros2_ws\src\my_first_pkg\my_first_pkg\run_both.py

# Turtle Sim:

ros2 run turtlesim turtlesim_node

ros2 run turtlesim turtle_teleop_key

ros2 node list

ros2 node info /turtlesim

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
