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
            "positioning_error = ros2_px4_testing.positioning_error:main",
            "video_streamer = ros2_px4_testing.video_streamer:main",
            "drone_vehicle_uwb_positioning = ros2_px4_testing.drone_vehicle_uwb_positioning:main"
        ],
    },
)
