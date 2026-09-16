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
    install_requires=['setuptools', 'message_filters', 'scipy'],
    zip_safe=True,
    maintainer='Architect',
    maintainer_email='manager@college.ac.in',
    description='Autonomous vision-based navigation node for SIH drone pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_nav_node = navigation_brain.vision_nav_node:main',
            'state_machine_node = navigation_brain.state_machine_node:main',
            'vio_bridge_node = navigation_brain.vio_bridge_node:main',
            'gazebo_odom_adapter_node = navigation_brain.gazebo_odom_adapter_node:main',
            'sim_vio_node = navigation_brain.sim_vio_node:main',
            'landing_state_node = navigation_brain.landing_state_node:main',
            'anti_sway_filter = navigation_brain.anti_sway_filter:main',
            'altimeter_node = navigation_brain.altimeter_node:main',
            'mission_node = navigation_brain.mission_node:main',

        ],
    },
)