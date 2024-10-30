from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_feed'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dstokes',
    maintainer_email='23677562@sun.ac.za',
    description='This package is responsible for publishing the camera feed to the workspace and optionally displaying it.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = camera_feed.publisher:main',
            'subscriber = camera_feed.subscriber:main',
        ],
    },
)