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
        ('share/' + package_name + '/launch', ['launch/camera_detection.launch.py']),
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
            'thruster_control = auton_sub.scripts.thruster_control:main',
            'leak_sensor = auton_sub.scripts.leak_sensor:main',
            'object_detection = auton_sub.object_detection:main',
            'jetson_camera = auton_sub.jetson_camera:main',
            'navigation = auton_sub.scripts.navigation:main',
            'thruster_node = auton_sub.thruster_node:main',
            'leak_node = auton_sub.leak_node:main',
            'test_thruster = auton_sub.test_thruster:main',
            'mission_control = auton_sub.mission_control:main',
            'claw = auton_sub.claw:main',
        ],
    },
)