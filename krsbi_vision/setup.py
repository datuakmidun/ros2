from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'krsbi_vision'

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
        # Model weights (placeholder)
        (os.path.join('share', package_name, 'models'),
            glob('models/*.pt') + glob('models/*.onnx') + glob('models/README.md')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'ultralytics',  # YOLOv8
        'scipy',        # Kalman filter
        'filterpy',     # Advanced Kalman
    ],
    zip_safe=True,
    maintainer='datuakmidun',
    maintainer_email='datuakmidun@gmail.com',
    description='Computer vision package for KRSBI-B Soccer Robot with YOLOv8 and Kalman filter',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = krsbi_vision.camera_node:main',
            'omni_camera_node = krsbi_vision.omni_camera_node:main',
            'yolo_detector = krsbi_vision.yolo_detector:main',
            'ball_detector = krsbi_vision.ball_detector:main',
            'ball_tracker = krsbi_vision.ball_tracker:main',
            'robot_detector = krsbi_vision.robot_detector:main',
            'field_detector = krsbi_vision.field_detector:main',
            'vision_fusion = krsbi_vision.vision_fusion:main',
            'color_calibrator = krsbi_vision.color_calibrator:main',
        ],
    },
)
