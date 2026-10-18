from setuptools import find_packages, setup

package_name = 'speaking_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dennis.mayer1204@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'audioCreationWorker = speaking_service.AudioCreationWorkerNode:main',
            'audioPlayerWorker = speaking_service.AudioPlayerWorkerNode:main',
            'audioListenderWorker = speaking_service.AudioListenerWorkerNode:main',
            'audioTranscriberWorker = speaking_service.AudioTranscriberWorkerNode:main',
        ],
    },
)
