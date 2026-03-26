import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'skypal_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skypal',
    maintainer_email='skypal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heart_node = skypal_core.heart_node:main',
            'mission_commander = skypal_core.mission_commander:main',
            'yolo_obstacle_detector = skypal_core.yolo_obstacle_detector:main',
            'sonar_multiplex = skypal_core.sonar_multiplex:main',
            'qr_scanner_node = skypal_core.qr_scanner_node:main',
            'path_tracker_node = skypal_core.path_tracker_node:main'
        ],
    },
)
