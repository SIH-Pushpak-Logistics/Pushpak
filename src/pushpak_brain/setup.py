from setuptools import find_packages, setup

package_name = 'pushpak_brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Architect',
    maintainer_email='manager@college.ac.in',
    description='PUSHPAK guidance: sole writer of velocity setpoints',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pushpak_brain = pushpak_brain.brain_node:main',
        ],
    },
)
