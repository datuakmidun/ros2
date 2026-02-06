from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'krsbi_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='datuakmidun',
    maintainer_email='datuakmidun@gmail.com',
    description='High-level interface, configuration, and launch files for KRSBI-B Soccer Robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_monitor = krsbi_interface.system_monitor:main',
            'param_server = krsbi_interface.param_server:main',
        ],
    },
)
