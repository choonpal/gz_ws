from setuptools import setup
import os
from glob import glob

package_name = 'parking_gz_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='협동 주차로봇 제어부 가제보 검증',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_rigid_body_sync = parking_gz_sim.sim_rigid_body_sync:main',
            'waypoint_publisher = parking_gz_sim.waypoint_publisher:main',
            'cctv_map_builder = parking_gz_sim.cctv_map_builder:main',
            'map_astar_planner = parking_gz_sim.map_astar_planner:main',
        ],
    },
)
