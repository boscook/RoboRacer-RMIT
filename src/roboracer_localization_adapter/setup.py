from setuptools import find_packages, setup

package_name = 'roboracer_localization_adapter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone',
    maintainer_email='capstone@todo.todo',
    description='RoboRacer localization adapter for Autoware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_adapter = '
            'roboracer_localization_adapter.localization_adapter:main',
        ],
    },
)
