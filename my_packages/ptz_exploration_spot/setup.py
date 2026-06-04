from glob import glob
from setuptools import find_packages, setup

PKG_NAME = 'ptz_exploration_spot'

setup(
    name=PKG_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG_NAME]),
        ('share/' + PKG_NAME, ['package.xml']),
        (f'share/{PKG_NAME}/launch', glob('launch/*')),
        (f'share/{PKG_NAME}/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenz Killer',
    maintainer_email='killer@lolo.place',
    description='PTZ exploration for Spot',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ptz_wrapper=ptz_exploration_spot.ptz_wrapper:main',
            'spot_cam_wrapper=ptz_exploration_spot.spot_cam_wrapper:main',
        ],
    },
)
