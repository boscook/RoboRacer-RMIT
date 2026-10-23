import os
from glob import glob
from setuptools import setup

package_name = 'roboracer_autodrive_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone',
    maintainer_email='capstone@localhost',
    description='RoboRacer <-> AutoDRIVE digital twin adapter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_adapter = roboracer_autodrive_bridge.sim_adapter:main',
            'sim_odom_tf = roboracer_autodrive_bridge.sim_odom_tf:main',
        ],
    },
)
