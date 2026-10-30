from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_teleop_launch.py',
            'launch/hw2_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/hw2_config.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
        'pynput>=1.7.0',
        'numpy>=1.20.0',
        'PyYAML>=5.4',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='chl235@ucsd.edu',
    description='Robot motor controller with keyboard input and HW2 visual navigation for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = robot_control.motor_control:main',
            'keyboard_control = robot_control.keyboard_control:main',
            'terminal_keyboard_control = robot_control.terminal_keyboard_control:main',
            'calibrate = robot_control.calibrate:main',
            'waypoint_follower = robot_control.waypoint_follower:main',
            'calibrate_camera = robot_control.calibrate_camera:main',
            'landmark_detector = robot_control.landmark_detector:main',
            'pose_estimator = robot_control.pose_estimator:main',
            'closed_loop_follower = robot_control.closed_loop_follower:main',
            'plot_hw2_results = robot_control.plot_hw2_results:main',
        ],
    },
)
