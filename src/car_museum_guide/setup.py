from setuptools import setup
from glob import glob
import os

package_name = 'car_museum_guide'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') + glob('config/*.json')),
        
        # Install resource files (optional wake word files, etc.)
        (os.path.join('share', package_name, 'resource'), 
            glob('resource/*')),
        
        # Install scripts (worker processes)
        (os.path.join('share', package_name, 'scripts'), 
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='firas',
    maintainer_email='firasbf6@gmail.com',
    description='Car museum guide robot with AI-powered descriptions, improved error handling, and visitor detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_museum_guide_node = car_museum_guide.car_museum_guide_node:main',
            'speech_recognition_node = car_museum_guide.speech_recognition_node:main',
            'visitor_detection_node = car_museum_guide.visitor_detection_node:main',
        ],
    },
)