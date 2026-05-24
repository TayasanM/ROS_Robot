from setuptools import setup
from glob import glob
import os

package_name = 'rosmaster_m1'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TayasanM',
    maintainer_email='your_email@example.com',
    description='ROS 2 applications for ROSMaster M1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kcf_follow = rosmaster_m1.kcf_follow:main',
            'color_follow = rosmaster_m1.color_follow:main',
        ],
    },
)
