import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('test_robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf/test_robot.xacro')
    robot_name_in_urdf = 'test_robot'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    description_package_name = "test_robot_description"

    # Position and orientation
    position = [0.0, 0.0, 0.0]          # [X, Y, Z]
    orientation = [0.0, 0.0, 0.0]       # [Roll, Pitch, Yaw]
    entity_name = robot_name_in_urdf    # Base Name or robot
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'test_robot.xacro'
    urdf_path = os.path.join(get_package_share_directory('test_robot_description'), 'urdf', urdf_file_name)

    install_dir = get_package_prefix(description_package_name)
    gazebo_models_path = os.path.join(pkg_share, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        output="screen"
    )

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_share, 'worlds', 'empty.world'), ''],
            description='SDF world file'),

        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        
    ])