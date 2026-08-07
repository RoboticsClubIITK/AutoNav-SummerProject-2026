from glob import glob
from setuptools import find_packages, setup

package_name = 'arduino_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/firmware', glob('firmware/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi',
    maintainer_email='konarkshah.8.a@gmail.com',
    description='Arduino Nano serial bridge for differential-drive encoders and motor commands.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge = arduino_base.serial_bridge:main',
        ],
    },
)
