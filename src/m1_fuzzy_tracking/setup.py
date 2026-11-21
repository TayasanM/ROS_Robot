import os
from glob import glob
from setuptools import setup

package_name = 'm1_fuzzy_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Fuzzy behavior arbiter and voice tracking control for ROSMASTER M1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fuzzy_arbiter = m1_fuzzy_tracking.fuzzy_arbiter_node:main',
            'voice_control = m1_fuzzy_tracking.voice_control_node:main',
            'yolo_tracker = m1_fuzzy_tracking.yolo_tracker_node:main',
        ],
    },
)
