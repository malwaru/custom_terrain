import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob

def generate_terrain_items_list(urdf_dir:str):
    '''
    Generate robot state publisher for each terrain item

    Args:
        urdf_dir(str)   : The directory of the urdf files

    Returns:
        start_robot_state_publisher_cmds(list) : The list of robot state publisher commands
    
    '''
    terrain_items_list={
            'robot_name_in_model':[],
            'spawn_x_val':[],
            'spawn_y_val':[],
            'spawn_z_val':[],
            'spawn_yaw_val':[],
            'robot_description':[],
                    }
    start_robot_state_publisher_cmds=[]
    items=glob.glob(urdf_dir+'/*.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')    
    if len(items)==0:
        print(f'No terrain items found in the directory {urdf_dir}')

    for item in items:
        #Terrain item information for spwaning
        robot_name=os.path.basename(item).split('.')[0]
        terrain_items_list['robot_name_in_model'].append(os.path.basename(item).split('.')[0])
        terrain_items_list['spawn_x_val'].append('0.0')
        terrain_items_list['spawn_y_val'].append('0.0')
        terrain_items_list['spawn_z_val'].append('0.0')
        terrain_items_list['spawn_yaw_val'].append('0.0')
        terrain_items_list['robot_description'].append(os.path.basename(item).split('.')[0])

        #Terrain item robot description command
        with open(item, 'r') as infp:
            robot_desc = infp.read()

        urdf_params = {'robot_description': robot_desc ,'use_sim_time':use_sim_time}        
        start_robot_state_publisher_cmds.append( Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        namespace=robot_name,
        parameters=[urdf_params])
        )

    return start_robot_state_publisher_cmds,terrain_items_list

def generate_launch_description():
    ld=LaunchDescription()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
    world_type='world_type/medium_world'
    urdf_folder_path='urdf'

    pkg_terrain_description=get_package_share_directory('custom_gazebo_world')
    path_urdf=os.path.join(pkg_terrain_description,world_type,urdf_folder_path)
    start_robot_state_publisher_cmds,terrain_items=generate_terrain_items_list(path_urdf)


    spawn_items_cmds=[]
    
    for iter in range(len(terrain_items['robot_name_in_model'])):
        spawn_items_cmds.append(
                    Node(
                            package='gazebo_ros', 
                            executable='spawn_entity.py',
                            # namespace=terrain_items['robot_name_in_model'][iter],
                            arguments=['-entity', terrain_items['robot_name_in_model'][iter], 
                                        '-topic', terrain_items['robot_description'][iter]+'/robot_description',
                                            '-x', terrain_items['spawn_x_val'][iter],
                                            '-y', terrain_items['spawn_y_val'][iter],
                                            '-z', terrain_items['spawn_yaw_val'][iter],
                                            '-Y', terrain_items['spawn_yaw_val'][iter]
                                        ],
                                            output='screen')

                            )
                            


    ld.add_action(declare_use_sim_time_cmd)
    #Spawn the robot state publishers
    for start_robot_state_publisher_cmd in start_robot_state_publisher_cmds:
        ld.add_action(start_robot_state_publisher_cmd)
    #Spawn the terrain items
    for spawn_item_cmd in spawn_items_cmds:
        ld.add_action(spawn_item_cmd)



    
    return ld