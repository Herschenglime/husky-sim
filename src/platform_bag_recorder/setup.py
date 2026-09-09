from glob import glob

from setuptools import find_packages, setup

package_name = 'platform_bag_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nick',
    maintainer_email='nicklausw2006@gmail.com',
    description='Rosbag2 sensor recorder for the a200 and go1 platforms.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bag_recorder = platform_bag_recorder.bag_recorder_node:main',
        ],
    },
)
