from setuptools import find_packages, setup

package_name = 'roboracer_track_memory'

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
    maintainer='capstone',
    maintainer_email='capstone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lap_detector = roboracer_track_memory.lap_detector:main',
            'memory_manager = roboracer_track_memory.memory_manager:main',
            'trajectory_recorder = roboracer_track_memory.trajectory_recorder:main',
            'trajectory_follower = roboracer_track_memory.trajectory_follower:main',
            'safety_supervisor = roboracer_track_memory.safety_supervisor:main',
            'safe_control_adapter = roboracer_track_memory.safe_control_adapter:main',
            'mode_manager = roboracer_track_memory.mode_manager:main',
            'controller_selector = roboracer_track_memory.controller_selector:main',
        ],
    },
)
