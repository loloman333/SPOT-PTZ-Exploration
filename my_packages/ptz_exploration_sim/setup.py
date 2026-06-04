from setuptools import find_packages, setup
from glob import glob

PKG_NAME = 'ptz_exploration_sim'

setup(
    name=PKG_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG_NAME]),
        ('share/' + PKG_NAME, ['package.xml']),
        (f'share/{PKG_NAME}/launch', glob('launch/*')),
        (f'share/{PKG_NAME}/config', glob('config/*')),
        (f'share/{PKG_NAME}/urdf', glob('urdf/*')),
        (f'share/{PKG_NAME}/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenz Killer',
    maintainer_email='killer@lolo.place',
    description='PTZ Exploration Simulator package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptz_simulator=ptz_exploration_sim.ptz_simulator:main',
        ],
    },
)
