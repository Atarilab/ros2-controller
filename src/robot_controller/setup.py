from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atari',
    maintainer_email='victor.dhedin@tum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = robot_controller.controller_node:main',
            'action_publisher_node = robot_controller.action_publisher_node:main',
            'state_publisher_node = robot_controller.state_publisher_node:main',
            'state_viewer_node = robot_controller.state_viewer_node:main',
            'simulator_node = robot_controller.simulator_node:main',
        ],
    },
)