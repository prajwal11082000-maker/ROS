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

package.xml: Tells ROS 2 "I am a package", what I depend on (rclpy, std_msgs), and who maintains me.

setup.py: Standard Python build script. It tells ROS 2 how to install your python scripts so they become executables (ros2 run ...).

launch/: Stores scripts that automate starting multiple nodes at once.

### Talker Node (talker.py)
This node publishes data.

class Talker(Node): Inherits from the ROS 2 Node class.

self.create_publisher(String, 'chatter', 10):

Creates a publisher on the topic named 'chatter'.

Message type is String.

Queue size is 10 (keeps last 10 messages if network is busy).

self.create_timer(1.0, self.timer_cb): Calls timer_cb every 1.0 second.

timer_cb: 

The function that actually does the work:

Creates a String message.

Sets data to "hello [count]".

Publishes it using self.pub.publish(msg).

### Listener Node (listener.py)

This node subscribes to data.

class Listener(Node): Inherits from Node.

self.create_subscription(String, 'chatter', self.cb, 10):

Listens to the same topic 'chatter'.

When a message arrives, it calls the function self.cb.

cb(self, msg): 

The callback function.

Receives the msg.

Logs it to the console: I heard: "hello ..."

### Launch File (talker_listener.launch.py)

This script automates running both nodes.

LaunchDescription: A container for all actions.

Node(...): Describes a node to run.

package='my_first_pkg': Look in this package.

executable='talker': Run the entry point named talker (defined in setup.py).

It does the same for the listener.

### Data Flow

Start: You run ros2 launch my_first_pkg talker_listener.launch.py.

Launch: The launch system starts both the talker process and the listener process.

Publish: Every 1 second, talker creates a message "hello X" and sends it to the topic chatter.

Transport: ROS 2 middleware (DDS) carries this message over the network (or shared memory).

Subscribe: The listener is waiting for messages on chatter. It receives the message.

Callback: The listener executes its cb function and prints "I heard: hello X" to the console.

### Execution

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

## Robot Kinematics Basics

Every URDF describes the robot using links and joints:

Link = Physical rigid part
(wheel, chassis, arm segment, sensor body)

Joint = Connection between two links
(revolute, continuous, fixed, prismatic, etc.)

You must know:

How many rigid parts exist

How they are connected

Degrees of freedom (DoF)

Joint limits, axis, and type

## Coordinate Frames

Each link has its own origin coordinate frame (x, y, z).
URDF uses right-handed coordinate system.

You must define:

origin/pose of child link relative to parent

orientation (roll–pitch–yaw)

## Mass, Inertia, and Physical Properties

URDF needs physical info for simulation:

mass

inertia matrix

center of mass

## Geometry

You describe shape using:

Box

Cylinder

Sphere

Mesh (.stl / .dae)

## Sensors and Plugins

URDF does not simulate sensors —
but you can include:

<gazebo> tags (for lidar, camera, plugins)

## All Important URDF Tags & What They Mean

robot root tag : ROBOT name

link tag — describes each rigid part

inertial tag : Mass and inertia:

visual tag

collision

### geometry tag

Box, Cylinder, Sphere, Mesh.

### joint — connects two links

fixed:	no motion

revolute:	rotates within limits

continuous:	rotates unlimited (like a wheel)

prismatic:	linear sliding

floating:	6 DoF (rare)

planar:	motion in 2D plane

## URDF Creation Checklist (Always Follow This)

Step 1: Identify all robot links: (chassis, wheels, arms, sensors)

Step 2: Define geometry + visuals: (box, cylinder, sphere, mesh)

Step 3: Add mass + inertia: (store values or approximate)

Step 4: Add joints: (type, parent–child, axis, limits)

Step 5: Ensure correct origins and alignment: (joint origin is the MOST important part!)

Step 6: Add transmissions (if using ros2_control) : Required if you want to load controller:

Step 7: Add Gazebo sensor/drive plugins

### How to run

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

A robot (simulated or real) publishing /scan (LIDAR) and /tf.

Teleoperation node (to drive the robot manually).

enmac@enmac-OptiPlex-5060:~$ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*

enmac@enmac-OptiPlex-5060:~$ sudo apt install ros-humble-slam-toolbox

enmac@enmac-OptiPlex-5060:~$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

enmac@enmac-OptiPlex-5060:~$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

enmac@enmac-OptiPlex-5060:~$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

enmac@enmac-OptiPlex-5060:~$ ros2 run teleop_twist_keyboard teleop_twist_keyboard

enmac@enmac-OptiPlex-5060:~$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

enmac@enmac-OptiPlex-5060:~$ ros2 run nav2_map_server map_saver_cli -f my_map

# Nav2

a highly modular system designed to move a robot from Point A to Point B safely and efficiently while avoiding obstacles.

The Workflow:

Localization: The robot figures out where it is (AMCL/SLAM).

Global Planning: It calculates a long path from start to goal (like Google Maps).

Local Planning (Control): It calculates immediate velocity commands (cmd_vel) to follow that path while dodging sudden obstacles (like a driver steering).

## Types of Maps in Nav2

Nav2 uses several "layers" of maps to understand the world. It doesn't just use one static image.

### Static Map (Occupancy Grid):

Format: Usually a .pgm (image) and .yaml (metadata) file.

The permanent map of walls and furniture created using SLAM. White pixels are free space, black are walls, and gray is unknown.

### Global Costmap:

Description: A map used by the Global Planner to calculate the long-term path. It inflates the size of walls based on the robot's radius so the robot doesn't plan a path too close to a wall.

### Local Costmap:

Description: A smaller, rolling window (e.g., 3x3 meters) that moves with the robot. It updates in real-time using LIDAR/Depth Camera data to detect dynamic obstacles (like people or pets) that aren't on the static map.

### Costmap Filters (Masks):

Keep-out Zones: A map layer used to strictly forbid the robot from entering certain areas (e.g., wet floors, steep drops).

Speed Limit Zones: A map layer that tells the robot to slow down in specific areas (e.g., a warehouse intersection).

## Important Algorithms in Nav2
To master Nav2, you must understand that "Planning" is split into two parts: Global (Path) and Local (Control).

A. Global Planners (Path Finding)

NavFn (Dijkstra/A*): The classic, reliable algorithm. Finds the mathematically shortest path.

Smac Planner: A more advanced planner that supports:

Hybrid-A*: Good for car-like (Ackermann) robots that can't turn in place.

State Lattice: Generates smooth, kinematically feasible paths.

B. Controllers / Local Planners (Path Following)

DWB (Dynamic Window Approach): The standard. It simulates multiple short trajectories and picks the one that makes the most progress without hitting obstacles.

MPPI (Model Predictive Path Integral): A modern, high-performance controller that uses GPU/CPU to simulate thousands of trajectories. excellent for dynamic environments and avoiding "jerky" movements.

RPP (Regulated Pure Pursuit): Simple and efficient, often used for Ackermann (car-like) steering robots.

## Localization & Mapping

AMCL (Adaptive Monte Carlo Localization): The algorithm used to track the robot's position on an existing map using particle filters.

SLAM Toolbox: The recommended algorithm in ROS 2 Humble for creating maps. It is graph-based and robust.

## Key Concepts (Nav2)

### Behavior Trees (BT):

Nav2 logic is defined in XML files. Instead of writing C++ code to change how the robot behaves (e.g., "If path is blocked, wait 5 seconds, then back up"), you edit the Behavior Tree XML. Understanding Sequence, Fallback, and Recovery nodes is crucial.

### Lifecycle Nodes:

Nav2 nodes (planner, controller, map_server) are "Lifecycle" nodes. They don't just "start"; they go through states: Unconfigured → Inactive → Active. You must "configure" and "activate" them (usually handled by the nav2_lifecycle_manager).

### Costmap Inflation & Footprint:

Tuning the inflation_radius and robot_footprint is the #1 reason why navigation fails. If the inflation is too high, the robot thinks it can't fit through a door. If too low, it scrapes the walls.

### TF (Transforms):

You must have a perfect TF tree (map -> odom -> base_link -> lidar_link). If your TFs are broken, Nav2 will refuse to move.
