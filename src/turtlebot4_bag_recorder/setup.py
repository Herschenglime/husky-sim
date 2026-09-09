import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot4_bag_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Records selected TurtleBot4 topics to a rosbag2 bag.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bag_recorder_node = turtlebot4_bag_recorder.bag_recorder_node:main',
        ],
    },
)
