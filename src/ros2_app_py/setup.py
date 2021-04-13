from setuptools import setup

package_name = 'ros2_app_py'

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
    maintainer_email='gennscar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "local_position_listener = ros2_app_py.local_position_listener:main",
            "offboard_control = ros2_app_py.offboard_control:main",
            "test = ros2_app_py.test:main",
            "target_position_publisher = ros2_app_py.target_position_publisher:main",
            "drone_controller = ros2_app_py.drone_controller:main"
            
        ],
    },
)
