# Created with Claude (Anthropic) on the basis of the car_museum_guide package.
from glob import glob

from setuptools import find_packages, setup

package_name = 'driving_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dennis Mayer',
    maintainer_email='dennis.mayer1204@gmail.com',
    description='Receptionist bot driving: MoveToLocation server and LiDAR visitor detector',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigatorNode = driving_service.navigator_node:main',
            'visitorDetectorNode = driving_service.visitor_detector_node:main',
        ],
    },
)
