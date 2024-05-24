from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Define the log directory and bag record paths
    log_directory = '/log'
    bag_directory = '/bag'

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

    # Configure Launch parameters
    uav_id = LaunchConfiguration('UAV_ID')
    mdf_file_path = LaunchConfiguration('MDF_FILE_PATH')
    sim = LaunchConfiguration('sim')

    # Define the ROS 2 bag recording process
    ros2_bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-e', 'uav_', '-o', bag_directory],
        additional_env={'ROS_LOG_DIR': log_directory},
        output='screen',
        on_exit=Shutdown()  # Ensures this process also shuts down when the launch is terminated
    )

    return LaunchDescription([
        uav_id_arg,
        mdf_file_path_arg,
        sim_arg,

        # Start the ROS 2 bag recording process
        ros2_bag_record,

        # Node definitions
        Node(
            package='waypoint_package',
            executable='waypoint_node',
            output='log'
            # arguments=['--ros-args', '--log-file', PathJoinSubstitution([log_directory, 'waypoint_node.log'])]
        ),
        Node(
            package='mission_control_package',
            executable='mission_control_node',
            output='log',
            parameters=[{'MDF_FILE_PATH': mdf_file_path}]
            # arguments=['--ros-args', '--log-file', PathJoinSubstitution([log_directory, 'mission_control_node.log'])]
        ),
        Node(
            package='qrcode_detection_package',
            executable='qr_code_scanner_node',
            output='log',
            parameters=[{'sim': sim}]
            # arguments=['--ros-args', '--log-file', PathJoinSubstitution([log_directory, 'qr_code_scanner_node.log'])]
        ),
        Node(
            package='fcc_bridge',
            executable='fcc_bridge',
            output='log',
            parameters=[{'UAV_ID': uav_id}]
            # arguments=['--ros-args', '--log-file', PathJoinSubstitution([log_directory, 'fcc_bridge.log'])]
        )
    ])
