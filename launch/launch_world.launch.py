import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
 

def generate_launch_description():
    ld=LaunchDescription()

    pkg_gazebo_ros=get_package_share_directory('gazebo_ros')
    pkg_gazebo_world=get_package_share_directory('custom_gazebo_world')
    pkg_terrain_description=get_package_share_directory('custom_gazebo_world')
    world_file=os.path.join(pkg_gazebo_world,'src/world','terrain.world')
    world = LaunchConfiguration('world')

    # Gazebo launch the empty world

    declare_world_file_cmd=DeclareLaunchArgument('world',default_value=world_file,description='Full path to world model file to load')
    start_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
     launch_arguments={'world': world}.items())
#     start_gazebo_server_cmd = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
#     launch_arguments={'world': world}.items())
 
#   # Start Gazebo client    
#     start_gazebo_client_cmd = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
#     )

    # Launch the terrain spawner launch file
    spawn_terrain=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                pkg_terrain_description + '/launch/launch_terrain_items.launch.py'))

    # spawn_terrain=PythonLaunchDescriptionSource(os.path.join(pkg_terrain_description,'launch','launch_terrain_items.launch.py'))




    
    ld.add_action(declare_world_file_cmd)
    ld.add_action(start_gazebo)
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_terrain)


    
    



    return ld