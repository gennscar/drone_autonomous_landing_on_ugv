import os
from setuptools import setup
from glob import glob

package_name = 'ros2_px4_swarming'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dellwork1',
    maintainer_email='dellwork1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'targetRover = ros2_px4_swarming.targetRover:main',
            'anchorDrone = ros2_px4_swarming.anchorDrone:main',
            'unitVectorsCalculator = ros2_px4_swarming.unitVectorsCalculator:main',
            'trackingVelocityCalculator = ros2_px4_swarming.trackingVelocityCalculator:main',
            'performanceAnalyzer = ros2_px4_swarming.performanceAnalyzer:main'
        ],
    },
)
