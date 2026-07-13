from glob import glob

from setuptools import find_packages, setup

package_name = 'fsai_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MDX FSAI Software Team',
    maintainer_email='mihiranra@gmail.com',
    description='Vehicle odometry (wheel + IMU dead-reckoning) and fixed-frame localization for the MDX FSAI stack.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'vehicle_odometry = fsai_localization.vehicle_odometry:main',
        ],
    },
)
