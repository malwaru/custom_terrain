import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():
    ld=LaunchDescription()

    pkg_gazebo_ros=get_package_share_directory('gazebo_ros')
    pkg_gazebo_world=get_package_share_directory('custom_terrain')
    pkg_terrain_description=get_package_share_directory('custom_terrain')
    world_file=os.path.join(pkg_gazebo_world,'src/worlds','terrain.world')

    # Gazebo launch the empty world

    declare_world_file_cmd=DeclareLaunchArgument('world_file',default_value=world_file,description='Full path to world model file to load')
    start_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),)

    # Launch the terrain spawner launch file
    spawn_terrain=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                pkg_terrain_description + '/launch/launch_terrain_items.launch.py'))

    # spawn_terrain=PythonLaunchDescriptionSource(os.path.join(pkg_terrain_description,'launch','launch_terrain_items.launch.py'))




    
    ld.add_action(declare_world_file_cmd)
    ld.add_action(start_gazebo)
    ld.add_action(spawn_terrain)


    
    



    return ld