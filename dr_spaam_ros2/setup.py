import os
from glob import glob
from setuptools import setup

package_name = 'dr_spaam_ros2'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dan Jia',
    maintainer_email='jia@vision.rwth-aachen.de',
    description='ROS2 wrapper package for 2D LiDAR person detection using DROW3 and DR-SPAAM',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dr_spaam_ros2_node = dr_spaam_ros2.dr_spaam_ros2_node:main'
        ],
    },
)
