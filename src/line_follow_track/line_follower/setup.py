from glob import glob

from setuptools import find_packages, setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nick',
    maintainer_email='nicklausw2006@gmail.com',
    description='OpenCV camera line follower for the painted warehouse track.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower = line_follower.follower:main',
            'score_run = line_follower.score_run:main',
        ],
    },
)
