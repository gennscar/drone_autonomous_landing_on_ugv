from setuptools import setup

package_name = 'ros2_px4_control'

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
            "hand_of_god_nav = ros2_px4_control.hand_of_god_nav:main",
            "drone_controller = ros2_px4_control.drone_controller:main",
            "drone_controller_old = ros2_px4_control.drone_controller_old:main",
            "rover_controller = ros2_px4_control.rover_controller:main",
            "offboard_control = ros2_px4_control.offboard_control:main"
        ],
    },
)
