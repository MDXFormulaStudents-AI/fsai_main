from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fsai_visualisation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/config',
            glob('config/*.rviz')),
        # Cone meshes
        ('share/' + package_name + '/meshes/cones',
            glob('meshes/cones/*')),
        # Car meshes
        ('share/' + package_name + '/meshes/car',
            glob('meshes/car/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MDX FSAI',
    maintainer_email='shuaibuoluwatunmise@gmail.com',
    description='RViz visualisation for MDX FSAI perception pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_visualizer = fsai_visualisation.cone_visualizer:main',
        ],
    },
)
