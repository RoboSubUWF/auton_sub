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
        ('share/' + package_name + '/launch', ['launch/auton_sub_launch.py']),
        ('share/' + package_name + '/launch', ['launch/mavros_launch.py']),
        ('share/' + package_name + '/launch', ['launch/dvl_mavros_launch.py']),
        ('share/' + package_name + '/launch', ['launch/manual_launch.py']),
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
            'servo = auton_sub.motion.servo:main',
            
        ],
    },
)