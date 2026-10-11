# ROS 2 `mobile_robot_lab_python` package의 설치 대상과 console executable을 정의한다.
# Launch·config·map resource를 ament share에 복사하고 Python node의 `main()`을 command로 등록한다.
# `colcon build`가 이 metadata를 읽어 `/ws/install` overlay에 package를 설치한다.
# Runtime sensor data를 처리하지 않으며 package version과 dependency metadata만 제공한다.

import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'mobile_robot_lab_python'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Mobin practice',
    maintainer_email='maintainer@example.com',
    description='Python learning nodes for the Mobin mobile robot lab.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_py = '
            'mobile_robot_lab_python.obstacle_avoidance:main',
            'lidar_bbox_association = '
            'mobile_robot_lab_python.lidar_bbox_association_node:main',
            'yolo_segmentation_detector = '
            'mobile_robot_lab_python.yolo_segmentation_detector_node:main',
            'warehouse_patrol = '
            'mobile_robot_lab_python.warehouse_patrol_node:main',
            'tracking_evaluator = '
            'mobile_robot_lab_python.tracking_evaluator_node:main',
            'trial_evidence_recorder = '
            'mobile_robot_lab_python.trial_evidence_recorder:main',
        ],
    },
)
