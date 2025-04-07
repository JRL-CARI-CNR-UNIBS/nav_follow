from launch import LaunchDescription
from launch_ros.actions import Node,SetRemap
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    declared_arguments = []


    log_level = LaunchConfiguration('log_level')

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    config = os.path.join(
        get_package_share_directory('nav_follow'),
        'config',
        'pid.yaml'
        )

    node1 = [Node(
                package='nav_follow',
                executable='nav_follow_node',
                output='screen',
                # prefix=['xterm -e gdb -ex run --args'],
                # prefix=['valgrind --tool=memcheck --leak-check=yes --show-reachable=yes '],
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{"laser_scan_slave": "/azrael/scan"},
                            {"laser_scan_master": "/omron/scan"},
                            {"cmd_vel_topic": "/azrael/cmd_vel"},
                            {"frame_name_laser_slave":  "azrael/laser"},
                            {"frame_name_laser_master": "omron/base_link"},
                            {"frame_name_base_slave":   "azrael/base_link"},
                            {"frame_name_base_master":  "omron/base_link"},
                            {"icp_iterations"                     : 100},
                            {"icp_TransformationEpsilon"          : 1e-10},
                            {"icp_EuclideanFitnessEpsilon"        : 0.00001},
                            {"icp_RANSACOutlierRejectionThreshold": 1.5},
                            {"icp_MaxCorrespondenceDistance"      : 100.0},
                            {"use_sim_time"                       : False},                
                            {"enable_tf"                          : True},
                            {"enable_vel_feedforward"             : False},
                            {"enable_icp"                         : False}, 
                            {"cmd_vel_topic_master"               : "/leader_simulation/cmd_vel"},
                            # {"cmd_vel_topic_master"               : "/omron/cmd_vel"},
                            {"autostart"                          : True},
                            config])]

    return LaunchDescription(declared_arguments + node1)

    