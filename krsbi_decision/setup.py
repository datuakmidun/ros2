from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'krsbi_decision'

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
    install_requires=['setuptools', 'py_trees', 'numpy'],
    zip_safe=True,
    maintainer='datuakmidun',
    maintainer_email='datuakmidun@gmail.com',
    description='Game strategy and behavior trees for KRSBI-B',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy_manager = krsbi_decision.strategy_manager:main',
            'game_controller = krsbi_decision.game_controller:main',
        ],
    },
)
