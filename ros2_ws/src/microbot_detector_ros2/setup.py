from setuptools import setup
import os
from glob import glob

package_name = 'microbot_detector_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for detecting and tracking microbots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'microbot_detector_node = microbot_detector_ros2.microbot_detector_node:main',
            'camera_calibration = microbot_detector_ros2.camera_calibration:main',
        ],
    },
)
