from setuptools import find_packages, setup

package_name = 'auton_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='kbenton4001@gmail.com',
    description='Sub',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_control = auton_sub.thruster_node:main',
            'leak_sensor = auton_sub.leak_node:main',
            'object_detection = auton_sub.vision_node:main',
            'navigation = auton_sub.navigation:main',
        ],
    },
)
