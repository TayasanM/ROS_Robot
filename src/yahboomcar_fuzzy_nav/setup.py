import os
from glob import glob
from setuptools import setup

package_name = 'yahboomcar_fuzzy_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Standalone Fuzzy Navigation Package for Yahboom M1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'fuzzy_tracker = yahboomcar_fuzzy_nav.fuzzy_tracker:main',
        'fuzzy_tracker_nv = yahboomcar_fuzzy_nav.fuzzy_tracker_nv:main',
        'voice_control = yahboomcar_fuzzy_nav.voice_control_node:main',
        'yolo_tracker = yahboomcar_fuzzy_nav.yolo_tracker_node:main',
    ],
},
)
