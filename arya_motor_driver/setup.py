from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arya_motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # === ROS 2 package index ===
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # === Package.xml ===
        ('share/' + package_name, ['package.xml']),

        # === Config folder ===
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # === Launch folder (semua launch file diikutkan) ===
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'crcmod',
        'pymodbus',
        'tf-transformations'
    ],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='rodik.w.i@ftmm.unair.ac.id',
    description='Motor driver and odometry bridge for ARYA AMR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = arya_motor_driver.motor_node:main',
            'odom_bridge = arya_motor_driver.odom_bridge:main',
        ],
    },
)
