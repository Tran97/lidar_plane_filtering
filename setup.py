from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'lidar_plane_filtering'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tadashi',
    maintainer_email='steven.t@outlook.dk',
    description='Filters lidar data based on orientation from imu',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_data_converter = lidar_plane_filtering.imu_converter:main',
            'plane_filter = lidar_plane_filtering.plane_filter:main',
            'orientation_tf = lidar_plane_filtering.orientation_tf:main',
            'lidar_tf = lidar_plane_filtering.lidar_tf:main',
            'pc2 = lidar_plane_filtering.pc2_test:main',
        ],
    },
)
