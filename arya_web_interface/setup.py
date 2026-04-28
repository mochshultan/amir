from setuptools import setup
import os
from glob import glob

package_name = 'arya_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Resource index agar ROS 2 mengenali package ini
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # File package.xml wajib untuk ROS 2
        ('share/' + package_name, ['package.xml']),

        # Semua launch file
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Semua file di folder static (rekursif)
      #  (os.path.join('lib', package_name, 'static'),
       #     glob('arya_web_interface/static/**/*', recursive=True)),
        
        # salin semua file dari static
        (os.path.join('share', package_name, 'static'),
            [f for f in glob('arya_web_interface/static/**/*', recursive=True)
             if os.path.isfile(f)]),

        # salin semua file dari templates
        (os.path.join('share', package_name, 'templates'),
            [f for f in glob('arya_web_interface/templates/**/*', recursive=True)
             if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='amr@example.com',
    description='Web interface for ARYA robot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'web_node = arya_web_interface.web_node:main',
        ],
    },
)
