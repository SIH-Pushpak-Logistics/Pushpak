import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'navigation_brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'message_filters'],
    zip_safe=True,
    maintainer='Architect',
    maintainer_email='manager@college.ac.in',
    description='Autonomous vision-based navigation node for SIH drone pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altimeter_node = navigation_brain.altimeter_node:main',
            'perception_node = navigation_brain.perception_node:main',
            'sim_radar_emulator_node = navigation_brain.sim_radar_emulator_node:main',
            'vio_bridge_node = navigation_brain.vio_bridge_node:main',
            'arm_takeoff_handshake = navigation_brain.arm_takeoff_handshake:main',
        ],
    },
)
