from glob import glob

from setuptools import find_packages, setup


package_name = 'fsai_mission_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/profiles', glob('profiles/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shuaibu Oluwatunmise',
    maintainer_email='shuaibuoluwatunmise@gmail.com',
    description='Mission supervision and static inspection profile execution for the MDX FSAI stack.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mission_manager = fsai_mission_control.mission_manager:main',
            'static_profile_executor = fsai_mission_control.static_profile_executor:main',
        ],
    },
)
