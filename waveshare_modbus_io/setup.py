from setuptools import find_packages, setup

package_name = 'waveshare_modbus_io'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/io_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/io.launch.py']),
    ],
    install_requires=['setuptools', 'pymodbus', 'pyserial'],
    zip_safe=True,
    maintainer='rodik',
    maintainer_email='rodik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'io_node = waveshare_modbus_io.io_node:main',
        ],
    },
)
