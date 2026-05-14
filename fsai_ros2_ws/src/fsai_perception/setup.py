from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fsai_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MDX FSAI',
    maintainer_email='shuaibuoluwatunmise@gmail.com',
    description='FSAI perception pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge          = fsai_perception.bridge:main',
            'lidar_detector  = fsai_perception.lidar_detector:main',
            'camera_detector = fsai_perception.camera_detector:main',
            'fusion          = fsai_perception.fusion:main',
        ],
    },
)
