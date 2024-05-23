from mission_software_launch.launch.launch import LaunchDescription
from launch_ros.actions import Node, ExecuteProcess

def generate_launch_description():

    # ROS 2 Parameter configurations for the nodes
    fcc_bridge_params = {'UAV_ID': 'SIMULATOR'}
    mission_control_params = {'MDF_FILE_PATH': 'DEFAULT'}
    qr_code_scanner_node_params = {'sim': True}

    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-e', 'uav_', '-o' , '~/records/rosbags'], 
            additional_env={'ROS_LOG_DIR' : '~/records/log_files'},
            output= 'screen'
        ), 

        Node(
            package='telemetry_package',
            executable='telemetry_node',
            name='telemetry',
            output='log'
            # output= 'screen',
            
        ),
        Node(
            package='waypoint_package',
            executable='waypoint_node',
            name='waypoint',
            output='log'
            # output= 'screen',
        ),
        Node(
            package='mission_control_package',
            executable='mission_control_node',
            name='mission_control',
            output='log',
            # output= 'screen',
            parameters=[mission_control_params]
        ),
        Node(
            package='qrcode_detection_package',
            executable='qrcode_detection_node',
            name='qrcode_detection',
            output='log',
            # output= 'screen',
            parameters=[qr_code_scanner_node_params]
        ),
        Node(
            package='fcc_bridge_package',
            executable='fcc_bridge',
            name='fcc_bridge',
            output='log',
            # output= 'screen',
            parameters=[fcc_bridge_params]
        )
    ])
