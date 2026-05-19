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
            'corridor_midpoint_planner = fsai_navigation.corridor_midpoint_planner:main',
            'global_path_follower = fsai_navigation.global_path_follower:main',
            'object_list_global_planner = fsai_navigation.object_list_global_planner:main',
            'one_sided_offset_planner = fsai_navigation.one_sided_offset_planner:main',
            'persistent_hybrid_planner = fsai_navigation.persistent_hybrid_planner:main',
            'persistent_corridor_planner = fsai_navigation.persistent_corridor_planner:main',
            'reactive_marker_planner = fsai_navigation.reactive_marker_planner:main',
        ],
    },
)
