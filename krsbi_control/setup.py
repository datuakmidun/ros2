from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'krsbi_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='datuakmidun',
    maintainer_email='datuakmidun@gmail.com',
    description='Motion control, kinematics, and path planning for KRSBI-B Soccer Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_controller = krsbi_control.motion_controller:main',
            'localization = krsbi_control.localization:main',
            'path_planner = krsbi_control.path_planner:main',
            'behavior_node = krsbi_control.behavior_node:main',
            'trajectory_tracker = krsbi_control.trajectory_tracker:main',
        ],
    },
)
