from setuptools import find_packages, setup

package_name = 'server_connector'

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
    maintainer='Dennis Mayer',
    maintainer_email='dennis.mayer1204@gmail.com',
    description='Gateway node that holds the connection to the external server for all other nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "serverConnectorNode = server_connector.ConnectorNode:main"
        ],
    },
)
