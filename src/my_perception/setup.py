from setuptools import find_packages, setup

package_name = 'my_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='yahboom@todo.todo',
    description='YOLO 3D Grounding and Perception Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_grounding = my_perception.yolo_grounding:main',
            'yolo_track_map_pid = my_perception.yolo_track_map_pid:main',
            'cmd_vel_mux = my_perception.cmd_vel_mux:main',
        ],
    },
)
