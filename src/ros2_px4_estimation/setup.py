from setuptools import setup

package_name = 'ros2_px4_estimation'

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
    maintainer=[
        'Cosimo Conte'
        'Gennaro Scarati'
        'Matteo Celada'
    ],
    maintainer_email=[
        'cosimocon@gmail.com'
        'gennscar97@gmail.com'
        'matte.celada@gmail.com'
    ],
    description='This package contains state estimation nodes for the drone or other components',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "uwb_positioning = ros2_px4_estimation.uwb_positioning:main",
            "px4_positioning = ros2_px4_estimation.px4_positioning:main",
            "kf_loose_positioning = ros2_px4_estimation.kf_loose_positioning:main",
            "ukf_positioning = ros2_px4_estimation.ukf_positioning:main",
            "uwb_estimate_2_px4 = ros2_px4_estimation.uwb_estimate_2_px4:main",
            "apriltag_yaw_estimator = ros2_px4_estimation.apriltag_yaw_estimator:main",
            "px4_yaw_estimator = ros2_px4_estimation.px4_yaw_estimator:main",
            "drone_rover_uwb_positioning = ros2_px4_estimation.drone_rover_uwb_positioning:main",
            "uwb_driver = ros2_px4_estimation.uwb_driver:main",
            "drone_rover_kf_pos = ros2_px4_estimation.drone_rover_kf_pos:main",
            "magnetometer_yaw_estimator = ros2_px4_estimation.magnetometer_yaw_estimator:main",
            "gazebo_yaw_estimator = ros2_px4_estimation.gazebo_yaw_estimator:main",
            "gps_positioning = ros2_px4_estimation.gps_positioning:main"
        ],
    },
)
