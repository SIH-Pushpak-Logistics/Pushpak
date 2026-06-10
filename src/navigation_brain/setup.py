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
    ],
    install_requires=['setuptools'],
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

        ],
    },
)