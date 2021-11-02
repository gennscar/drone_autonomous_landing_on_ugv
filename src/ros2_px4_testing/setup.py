from setuptools import setup

package_name = 'ros2_px4_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gennscar',
    maintainer_email='gennscar97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "odometry_error = ros2_px4_testing.odometry_error:main",
            "test_apriltag = ros2_px4_testing.test_apriltag:main",
            "setpoints_flight = ros2_px4_testing.setpoints_flight:main",
            "drone_rover_positioning_error = ros2_px4_testing.drone_rover_positioning_error:main",
            "rover_yaw_error = ros2_px4_testing.rover_yaw_error:main",
            "drone_rover_topic_recorder = ros2_px4_testing.drone_rover_topic_recorder:main",
            "compass_testing = ros2_px4_testing.compass_testing:main",
            "test_camera_raspberry = ros2_px4_testing.test_camera_raspberry:main",
        ],
    },
)
