from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'intro_to_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joshuatang',
    maintainer_email='joshuatang2007@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['publisher = intro_to_ros.publisher:main',
# 'subscriber = intro_to_ros.subscriber:main',
'bluerov2_sensors = intro_to_ros.bluerov2_sensors:main',
'arm = intro_to_ros.arm:main',
'movement = intro_to_ros.movement:main',
'depth = intro_to_ros.depth:main',
'depth_control = intro_to_ros.depth_control:main',
'heading_control = intro_to_ros.heading_control:main'
        ],
    },
)
