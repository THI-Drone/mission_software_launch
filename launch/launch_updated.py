from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from datetime import datetime

def generate_launch_description():
    # Timestamp
    timestamp = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
    log_directory = '/log/' + timestamp
    bag_directory = '/bag/' + timestamp

    # Command line arguments
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
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='uav_1',
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

    # ROS Bag record
    ros2_bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-e', 'uav_', '-o', bag_directory],
        output='screen',
        on_exit=Shutdown()
    )

    return LaunchDescription([
        uav_id_arg,
        mdf_file_path_arg,
        sim_arg,
        namespace_arg,
        set_ros_log_dir,

        ros2_bag_record,

        # Nodes with namespace set directly
        Node(
            package='waypoint_package',
            executable='waypoint_node',
            namespace=namespace,
            output='log'
        ),
        Node(
            package='mission_control_package',
            executable='mission_control_node',
            namespace=namespace,
            output='log',
            parameters=[{'MDF_FILE_PATH': mdf_file_path}]
        ),
        Node(
            package='qrcode_detection_package',
            executable='qr_code_scanner_node',
            namespace=namespace,
            output='log',
            parameters=[{'sim': sim}]
        ),
        Node(
            package='fcc_bridge',
            executable='fcc_bridge',
            namespace=namespace,
            output='log',
            parameters=[{'UAV_ID': uav_id}]
        )
    ])