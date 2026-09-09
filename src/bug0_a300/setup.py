import os
from glob import glob
from setuptools import setup

package_name = 'bug0_a300'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vaishnav Ramesh',
    maintainer_email='vaishnavr27@gmail.com',
    description='Bug0 navigation controller for the Clearpath A300 Observer (3D lidar).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug0_node = bug0_a300.bug0_node:main',
        ],
    },
)
