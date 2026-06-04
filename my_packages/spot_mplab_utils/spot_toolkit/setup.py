from setuptools import find_packages, setup
from glob import glob

package_name = 'spot_toolkit'

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
    maintainer_email='bancsi.mark.attila@sztaki.hu',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_relay = spot_toolkit.trajectory_relay:main',
            'velocity_relay = spot_toolkit.velocity_relay:main',
            'simple_spotty_commander = spot_toolkit.simple_spotty_commander:main',
            'bag_recorder = spot_toolkit.bag_recorder_gui:main',
        ],
    },
)
