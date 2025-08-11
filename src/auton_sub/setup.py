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
            'dvl_node = auton_sub.sensors.dvl_node:main',
            'dvl_mavros_bridge = auton_sub.sensors.dvl_mavros_bridge:main',
            'leak_node = auton_sub.sensors.leak_node:main',
            'test_thruster = auton_sub.test_thruster:main',
            'mission_control = auton_sub.mission_control:main',
            'claw = auton_sub.claw:main',
            'arm = auton_sub.utils.arm:main',
            'disarm = auton_sub.utils.disarm:main',
            'rosbag = auton_sub.utils.rosbag_launcher:main',
            'keyboard = auton_sub.utils.keyboard:main',
            'keyboard_guided = auton_sub.utils.keyboard_guided:main',
            'servo = auton_sub.motion.servo:main',
            'prequal = auton_sub.mission.prequal:main',
            'coin_toss = auton_sub.mission.coin_toss:main',
            'test_thrust = auton_sub.mission.test_thrust:main',
            'depth_hold = auton_sub.mission.depth_hold'
            
        ],
    },
)