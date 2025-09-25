from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rso2_lidar_scanner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jules',
    maintainer_email='dev@example.com',
    description='ROS 2 package for RSO2 RPLIDAR Scanner',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanner_subscriber = rso2_lidar_scanner.scanner_subscriber:main',
        ],
    },
)