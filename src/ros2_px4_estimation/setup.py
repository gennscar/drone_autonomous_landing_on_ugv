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
    maintainer='gennscar',
    maintainer_email='gennscar97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "uwb_positioning = ros2_px4_estimation.uwb_positioning:main",
            "px4_positioning = ros2_px4_estimation.px4_positioning:main",
            "kf_loose_positioning = ros2_px4_estimation.kf_loose_positioning:main",
            "kf_tight_positioning = ros2_px4_estimation.kf_tight_positioning:main",
            "ukf_positioning = ros2_px4_estimation.ukf_positioning:main",
            "video_streamer = ros2_px4_estimation.video_streamer:main",
            "drone_vehicle_uwb_positioning = ros2_px4_estimation.drone_vehicle_uwb_positioning:main",
            "drone_vehicle_kf_loose = ros2_px4_estimation.drone_vehicle_kf_loose:main",
            "uwb_driver = ros2_px4_estimation.uwb_driver:main",
            "drone_vehicle_px4_positioning = ros2_px4_estimation.drone_vehicle_px4_positioning:main"
        ],
    },
)
