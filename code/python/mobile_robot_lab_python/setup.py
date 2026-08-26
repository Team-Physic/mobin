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
        ],
    },
)
