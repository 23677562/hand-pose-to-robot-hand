from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

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
    description='This package contains the control nodes for robotic hand control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'allegro_hand_gesture_control = control.allegro_hand_gesture_control:main',
            'hand_pose_to_allegro_hand = control.hand_pose_to_allegro_hand:main',
            'hand_pose_to_joint_state = control.hand_pose_to_joint_state:main',
        ],
    },
)
