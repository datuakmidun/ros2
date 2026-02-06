from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'krsbi_description'

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
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Meshes
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='datuakmidun',
    maintainer_email='datuakmidun@gmail.com',
    description='URDF robot description and visualization for KRSBI-B Soccer Robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = krsbi_description.state_publisher:main',
        ],
    },
)
