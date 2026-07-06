from glob import glob

from setuptools import find_packages, setup

package_name = 'fsai_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i',
    maintainer_email='2dellab@gmail.com',
    description='Navigation experiments for Formula Student AI cone-based planning.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'acceleration_nav = fsai_navigation.acceleration_nav:main',
            'forward_distance_controller = fsai_navigation.forward_distance_controller:main',
            'ground_truth_path = fsai_navigation.ground_truth_path:main',
            'perceived_path = fsai_navigation.perceived_path:main',
            'persistent_path = fsai_navigation.persistent_path:main',
            'local_path_follower = fsai_navigation.local_path_follower:main',
            'pure_pursuit_path_follower = fsai_navigation.local_path_follower:pure_pursuit_main',
            'stanley_path_follower = fsai_navigation.stanley_path_follower:main',
            'global_path_follower = fsai_navigation.global_path_follower:main',
        ],
    },
)
