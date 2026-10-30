"""Launch file for CSE 276A HW2 Closed-Loop Visual Navigation.

Starts:
  1. motor_control
  2. landmark_detector (single YOLO model: `yolo_world` or `yolov8s`)
  3. pose_estimator (Range-Bearing EKF with known map)
  4. closed_loop_follower (Visual feedback waypoint controller + trajectory logger)

Usage:
    ros2 launch robot_control hw2_launch.py
    ros2 launch robot_control hw2_launch.py model_mode:=yolov8s unit_scale_m:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_cfg = PathJoinSubstitution([
        FindPackageShare('robot_control'),
        'config',
        'hw2_config.yaml',
    ])

    config_file_arg = DeclareLaunchArgument('config_file', default_value=default_cfg)
    model_mode_arg = DeclareLaunchArgument('model_mode', default_value='yolo_world')
    unit_scale_arg = DeclareLaunchArgument('unit_scale_m', default_value='0.5')
    dry_run_arg = DeclareLaunchArgument('dry_run', default_value='false')

    config_file = LaunchConfiguration('config_file')
    model_mode = LaunchConfiguration('model_mode')
    unit_scale_m = LaunchConfiguration('unit_scale_m')
    dry_run = LaunchConfiguration('dry_run')

    motor_node = Node(
        package='robot_control',
        executable='motor_control',
        name='motor_controller_node',
        output='screen',
        emulate_tty=True,
    )

    detector_node = Node(
        package='robot_control',
        executable='landmark_detector',
        name='landmark_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_file': config_file,
            'model_mode': model_mode,
        }],
    )

    ekf_node = Node(
        package='robot_control',
        executable='pose_estimator',
        name='pose_estimator_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_file': config_file,
            'unit_scale_m': unit_scale_m,
        }],
    )

    follower_node = Node(
        package='robot_control',
        executable='closed_loop_follower',
        name='closed_loop_follower_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_file': config_file,
            'unit_scale_m': unit_scale_m,
            'dry_run': dry_run,
        }],
    )

    return LaunchDescription([
        config_file_arg,
        model_mode_arg,
        unit_scale_arg,
        dry_run_arg,
        motor_node,
        detector_node,
        ekf_node,
        follower_node,
    ])
