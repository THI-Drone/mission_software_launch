import errno
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from datetime import datetime


def symlink_force(target, link_name, target_is_directory=False):
    try:
        os.symlink(target, link_name, target_is_directory)
    except OSError as e:
        if e.errno == errno.EEXIST:
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            raise e


def generate_launch_description():
    # Timestamp
    timestamp = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')

    log_directory = os.path.join('/log', timestamp)
    symlink_force(timestamp, os.path.join('/log', 'latest'), True)

    img_directory = os.path.join('/images', timestamp)
    symlink_force(timestamp, os.path.join('/images', 'latest'), True)

    # Command line arguments
    uav_id_arg = DeclareLaunchArgument(
        'UAV_ID',
        description='ID for the UAV'
    )
    mdf_file_path_arg = DeclareLaunchArgument(
        'MDF_FILE_PATH',
        description='Path to the Mission Definition File'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        description='Flag to run in simulation mode'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
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
            remappings=[('/rosout', 'rosout')],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='mission_control_package',
            executable='mission_control_node',
            namespace=namespace,
            output='log',
            parameters=[{'MDF_FILE_PATH': mdf_file_path}],
            remappings=[('/rosout', 'rosout')],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='qrcode_detection_package',
            executable='qr_code_scanner_node',
            namespace=namespace,
            output='log',  # Ensure logging to file
            parameters=[{'sim': sim, 'IMG_PATH': img_directory}],
            remappings=[('/rosout', 'rosout')],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='fcc_bridge',
            executable='fcc_bridge',
            namespace=namespace,
            output='log',
            parameters=[{'UAV_ID': uav_id}],
            remappings=[('/rosout', 'rosout')],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
