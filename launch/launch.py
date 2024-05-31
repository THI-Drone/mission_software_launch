import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from datetime import datetime


def generate_launch_description():
    # Timestamp
    timestamp = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')

    log_directory = os.path.join('/log', timestamp)
    bag_directory = os.path.join('/bag', timestamp)
    img_directory = os.path.join('/images', timestamp)

    # Command line arguments
    uav_id_arg = DeclareLaunchArgument(
        'UAV_ID',
        # default_value='SIMULATOR',
        description='ID for the UAV'
    )
    mdf_file_path_arg = DeclareLaunchArgument(
        'MDF_FILE_PATH',
        # default_value='DEFAULT',
        description='Path to the Mission Definition File'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        # default_value='True',
        description='Flag to run in simulation mode'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        # default_value='uav_1',
        description='Namespace for the UAV'
    )

    # Environment variable globally for all nodes
    set_ros_log_dir = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value=log_directory
    )

    # Launch parameters
    uav_id = LaunchConfiguration('UAV_ID')
    mdf_file_path = LaunchConfiguration('MDF_FILE_PATH')
    sim = LaunchConfiguration('sim')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        uav_id_arg,
        mdf_file_path_arg,
        sim_arg,
        namespace_arg,
        set_ros_log_dir,

        # Nodes
        Node(
            package='waypoint_package',
            executable='waypoint_node',
            namespace=namespace,
            output='log',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='mission_control_package',
            executable='mission_control_node',
            namespace=namespace,
            output='log',
            parameters=[{'MDF_FILE_PATH': mdf_file_path}], 
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='qrcode_detection_package',
            executable='qr_code_scanner_node',
            namespace=namespace,
            output='log',
            parameters=[{'sim': sim, 'IMG_PATH': img_directory}], 
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='fcc_bridge',
            executable='fcc_bridge',
            namespace=namespace,
            output='log',
            parameters=[{'UAV_ID': uav_id}], 
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
