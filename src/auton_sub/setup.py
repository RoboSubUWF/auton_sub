import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'auton_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['auton_sub', 'auton_sub.*', 'test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        #('share/' + package_name + '/launch', ['launch/auton_sub_launch.py']),
        #('share/' + package_name + '/launch', ['launch/mavros_launch.py']),
        #('share/' + package_name + '/launch', ['launch/dvl_mavros_launch.py']),
        #('share/' + package_name + '/launch', ['launch/pixhawk_launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='kbenton4001@gmail.com',
    description='Sub',
    license='Apache-2.0',
    #extras_require={
     #   'test': ['pytest'],
    #}
    entry_points={
        'console_scripts': [
            'object_detection = auton_sub.cv.object_detection:main',
            '120size = auton_sub.cv.120size:main',
            'front_multi = auton_sub.cv.front_multi:main',
            'bottom_multi = auton_sub.cv.bottom_multi:main',
            'headless_front = auton_sub.cv.headless_front:main',
            'dual_camera = auton_sub.cv.dual_camera:main',
            'dvl_node = auton_sub.sensors.dvl_node:main',
            'dvl_mavros_bridge = auton_sub.sensors.dvl_mavros_bridge:main',
            'leak_node = auton_sub.sensors.leak_node:main',
            'test_thruster = auton_sub.test_thruster:main',
            'mission_control = auton_sub.mission_control:main',
            'claw = auton_sub.claw',
            'claw_control_node = auton_sub.claw.claw_control_node:main',
            'claw_open = auton_sub.claw.claw_open:main',
            'claw_close = auton_sub.claw.claw_close:main',
            'arm = auton_sub.utils.arm:main',
            'disarm = auton_sub.utils.disarm:main',
            'rosbag = auton_sub.utils.rosbag_launcher:main',
            'keyboard = auton_sub.utils.keyboard:main',
            'keyboard_guided = auton_sub.utils.keyboard_guided:main',
            'servo = auton_sub.motion.servo:main',
            'prequal = auton_sub.mission.prequal:main',
            'coin_toss = auton_sub.mission.coin_toss:main',
            'gate = auton_sub.mission.gate:main',
            'test_thrust = auton_sub.mission.test_thrust:main',
            'depth_hold = auton_sub.mission.depth_hold:main',
            'full_planner = auton_sub.mission.full_planner:main',
            'Ocean_Cleanup = auton_sub.mission.Ocean_Cleanup:main',
            'slalom = auton_sub.mission.slalom:main',
            'path = auton_sub.mission.path:main',
            'home = auton_sub.mission.home:main',
        ],
    },
)