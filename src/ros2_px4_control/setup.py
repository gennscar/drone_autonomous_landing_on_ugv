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
    description='This package contains nodes to control the drone interfacing with PX4',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drone_controller = ros2_px4_control.drone_controller:main",
            "odometry_sender = ros2_px4_control.odometry_sender:main"
        ],
    },
)
