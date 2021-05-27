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
            "ins_positioning = ros2_px4_estimation.ins_positioning:main",
            "px4_positioning = ros2_px4_estimation.px4_positioning:main",
            "kf_positioning = ros2_px4_estimation.kf_loose_positioning:main",
            "video_streamer = ros2_px4_estimation.video_streamer:main",
            "drone_vehicle_uwb_positioning = ros2_px4_estimation.drone_vehicle_uwb_positioning:main",
            "kf_drone_rover = ros2_px4_estimation.kf_drone_rover:main",
            "uwb_driver = ros2_px4_estimation.uwb_driver:main"
        ],
    },
)
