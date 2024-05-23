from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from common_package_py.node_names import NodeNames

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
            package='waypoint_package',
            executable='waypoint_node',
            name=NodeNames.WAYPOINT,
            output='log'
        ),
        Node(
            package='mission_control_package',
            executable='mission_control_node',
            name=NodeNames.MISSION_CONTROL,
            output='log',
            parameters=[mission_control_params]
        ),
        Node(
            package='qrcode_detection_package',
            executable='qr_code_scanner_node',
            name=NodeNames.QRCODE_SCANNER,
            output='log',
            parameters=[qr_code_scanner_node_params]
        ),
        Node(
            package='fcc_bridge',
            executable='fcc_bridge',
            name=NodeNames.FCC_BRIDGE,
            output='log',
            parameters=[fcc_bridge_params]
        )
    ])
