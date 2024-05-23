from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare command line arguments
    uav_id_arg = DeclareLaunchArgument(
        'UAV_ID',
        default_value='SIMULATOR',
        description='ID for the UAV'
    )

    mdf_file_path_arg = DeclareLaunchArgument(
        'MDF_FILE_PATH',
        default_value='DEFAULT',
        description='Path to the Mission Definition File'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='True',
        description='Flag to run in simulation mode'
    )

    # Use LaunchConfiguration to use the command line arguments in node parameters
    uav_id = LaunchConfiguration('UAV_ID')
    mdf_file_path = LaunchConfiguration('MDF_FILE_PATH')
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        uav_id_arg,
        mdf_file_path_arg,
        sim_arg,

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-e', 'uav_', '-o', '~/records/rosbags'],
            additional_env={'ROS_LOG_DIR': '~/records/log_files'},
            output='screen'
        ),

        Node(
            package='waypoint_package',
            executable='waypoint_node',
            output='log'
        ),

        Node(
            package='mission_control_package',
            executable='mission_control_node',
            output='log',
            parameters=[{'MDF_FILE_PATH': mdf_file_path}]
        ),

        Node(
            package='qrcode_detection_package',
            executable='qr_code_scanner_node',
            output='log',
            parameters=[{'sim': sim}]
        ),

        Node(
            package='fcc_bridge',
            executable='fcc_bridge',
            output='log',
            parameters=[{'UAV_ID': uav_id}]
        )
    ])
