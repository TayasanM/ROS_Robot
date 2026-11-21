import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the launch directory:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Perception, tracking, and behavior arbitration for ROSMASTER M1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_mux = my_perception.cmd_vel_mux:main',
            'voice_tracker_node = my_perception.voice_tracker_node:main',
            'yolo_grounding = my_perception.yolo_grounding:main',
            'yolo_track_map_pid = my_perception.yolo_track_map_pid:main',
            'yolo_track_voice_pid = my_perception.yolo_track_voice_pid:main',
            'fuzzy_arbiter = my_perception.fuzzy_arbiter:main',
        ],
    },
)
