import os
from glob import glob
from setuptools import setup

package_name = 'moss_control'
submodule_name = 'moss_control/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='razeragon',
    maintainer_email='kward2787@gmail.com',
    description='Control package for the simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_corner = moss_control.gazebo_corner:main',
            'gazebo_estimator = moss_control.gazebo_estimator:main',
            'gazebo_square = moss_control.gazebo_swarm_square:main',
            'gazebo_vote = moss_control.gazebo_voter:main',
            'gazebo_movement = moss_control.gazebo_movement:main',
            'gazebo_camera = moss_control.gazebo_camera:main'
        ],
    },
)
