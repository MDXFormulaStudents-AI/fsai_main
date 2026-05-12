from setuptools import find_packages, setup

package_name = 'exercise'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task2_relay     = exercise.task2_relay:main',
            'task3_distance  = exercise.task3_distance:main',
            'task4_counter   = exercise.task4_counter:main',
            'task5_overlay   = exercise.task5_image_overlay:main',
        ],
    },
)
