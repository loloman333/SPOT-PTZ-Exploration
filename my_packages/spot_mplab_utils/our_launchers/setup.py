import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'our_launchers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')), #launchfiles
        ('share/' + package_name + '/config', glob('config/*')), #config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bancsi Márk',
    maintainer_email='bancsimark@sztaki.com',
    description='This package provides a set of ROS2 launchers for the spotty robot.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'time_fixer = our_launchers.lidar_time_fixer:main',
        ],
    },
)
