import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Names and Paths
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_dd_robot'
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/empty_world.world' # Placeholder
    
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
    # Process Xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    # Gazebo Logic
    gazeboRosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    gazeboLaunch = IncludeLaunchDescription(
        gazeboRosPackageLaunch,
        # launch_arguments={'world': pathWorldFile}.items() # Uncomment if using a world file
    )
    
    # Spawn Entity Node
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )
    
    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )
    
    ld = LaunchDescription()
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNode)
    ld.add_action(nodeRobotStatePublisher)
    
    return ld
