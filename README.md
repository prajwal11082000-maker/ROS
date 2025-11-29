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

"Where am I?" and "What does the world look like?"

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

# Gazebo (Gazebo Fortress)

SDF vs. URDF: ROS 2 uses URDF (Unified Robot Description Format) to describe robots. Gazebo uses SDF (Simulation Description Format).

The Trick: You don't need to write SDF manually. The ros_gz tools convert your URDF to SDF automatically in the background.

The Bridge (ros_gz_bridge): Unlike ROS 1, Gazebo is now a completely separate program from ROS. They talk via a "Bridge."

Gazebo publishes to a Gz Topic.

The Bridge listens to it and republishes it as a ROS 2 Topic.

Plugins: Code that makes things move.

Model Plugins: Control a specific robot (e.g., DiffDrive for wheels).

Sensor Plugins: Generate data (Lidar, Camera, IMU).

World Plugins: Control the environment (e.g., strict physics timing).

### How to Simulate Any Robot

1. Create a URDF file

Visual: What it looks like (mesh or shape).

Collision: Simplified shape (box/cylinder) for physics calculations. Crucial: High-poly meshes here will crash your simulation.

Inertial: Mass and inertia matrix.

2. The Gazebo Configuration
   
Add the ros_gz plugin to your URDF to tell Gazebo how to control the robot.

3. The Bridge
  
You need a YAML file or launch arguments to bridge topics.

Example Bridge: cmd_vel (ROS) $\rightarrow$ cmd_vel (Gazebo)

Example Bridge: scan (Gazebo) $\rightarrow$ scan (ROS)

4.The Launch File

Create a Python launch file (simulation.launch.py) that executes these 3 nodes simultaneously:

ros_gz_sim: Starts the Gazebo world.

robot_state_publisher: Publishes your URDF to ROS.

create (from ros_gz_sim): Spawns your robot into the Gazebo world.

### concepts i need to explore

Headless Mode: Running the simulation without the 3D graphics window. This saves massive computing power for training AI or running CI/CD tests.

Physics Engines: Gazebo isn't just one engine. You can swap the underlying math engine (DART, Bullet, ODE) depending on if you need accuracy (grasping) or speed (navigation).

Hardware Acceleration: Using your GPU to render sensor data (Lidar/Camera) instead of your CPU.

Physics Parameters (The "Realism vs. Speed" Trade-off)

Sensor Noise (The "Reality Gap"): real sensors are imperfect

The "Shadows" and the foot prints.

### next steps

1.The Mathematical Fixers: Filters vs. Graphs

To fix the drift from odometry, we use mathematical algorithms to fuse data from different sensors. There are two main approaches: Filtering (Real-time, "Here and Now") and Graph Optimization (Global, "Past and Present").

A. Kalman Filters (The "Filter" Approach)
Used primarily for Localization (fusing Odometry + IMU).

B. Kalman Filter (KF): Mathematical magic that takes two noisy guesses (e.g., "Wheel says I moved 1m," "GPS says I moved 1.2m") and merges them based on how trustworthy each sensor is (Covariance) to give a result better than either alone. It assumes linear motion.

C. Extended Kalman Filter (EKF): The industry standard for robots. Because robots drive in curves (non-linear), the EKF uses calculus (Jacobians) to linearize the math at the current point.

Why use it: It is fast and computationally cheap.

When to use it: To fuse IMU and Wheel Encoders to get a smooth odom -> base_link transform.

2. Graph Optimization (The "Global" Approach)
   
Used primarily for SLAM (Building the map).

A. Pose Graph: Imagine every time the robot stops to scan, it drops a "node" (a pose). It connects these nodes with "springs" (constraints based on odometry). If the robot returns to a spot it has seen before (Loop Closure), it connects the new node to the old node. The algorithm then "relaxes" the springs, bending the whole path to fit perfectly.

B. Factor Graph: A more generalized/advanced version of a Pose Graph. Instead of just relating Poses to Poses, it relates Factors (raw measurements, GPS points, landmarks) to Poses. It allows for more complex sensor fusion.

Why use it: It is more accurate than filters for large maps because it can correct past mistakes. Filters can only correct the current state.

3. SLAM (Simultaneous Localization and Mapping)
   
What is it? The robot enters an unknown room. It must build a map of the room while simultaneously keeping track of where it is inside that growing map. 

How it minimizes uncertainty: It uses the Loop Closure technique. When the robot recognizes a place it has been before, it "snaps" the map together, eliminating the drift that accumulated while exploring.

Types of SLAM:
Lidar SLAM (2D):

GMapping: (Old, Particle Filter based).

Cartographer: (Google, Graph-based).

SLAM Toolbox: (The standard for ROS 2). Uses Pose Graph Optimization (Karto). It is essentially a highly optimized Factor Graph solver.

Visual SLAM (vSLAM): Uses cameras to track "features" (corners, edges). Examples: ORB-SLAM, RTAB-Map.

Visual-Inertial Odometry (VIO): Combines Camera + IMU (often using EKF or Factor Graphs) for drones or walking robots.

# Road Map

Level 1: Hardware Driver

robot hardware publishes raw /odom (noisy) and /imu (noisy).

Level 2: Local EKF (robot_localization)

run the ekf_node.

Input: /odom (wheels) + /imu_data.

Output: /odometry/filtered.

Result: smooth coordinate frame called odom. The robot moves smoothly in Rviz.

Level 3: Global SLAM (slam_toolbox)

run slam_toolbox.

Input: /scan + /odometry/filtered (from the EKF).

Process: It builds a Pose Graph. When you drive a loop, it optimizes the graph.

Output: The /map frame and the map image.
