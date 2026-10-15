from setuptools import setup

package_name = 'roboracer_track_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone',
    maintainer_email='capstone@localhost',
    description='Live LiDAR track detector for RoboRacer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'track_detector = '
            'roboracer_track_detector.track_detector:main',
            'track_controller = '
            'roboracer_track_detector.track_controller:main',
            'track_controller_max = '
            'roboracer_track_detector.track_controller_max:main',
        ],
    },
)
