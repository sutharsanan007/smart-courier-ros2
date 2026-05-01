from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'osm_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pins',
    maintainer_email='pins@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'osm_planner = osm_navigator.osm_planner:main',
            'synthetic_gps = osm_navigator.synthetic_gps:main',
            'imu_bridge = osm_navigator.imu_bridge:main',
        ],
    },
)
