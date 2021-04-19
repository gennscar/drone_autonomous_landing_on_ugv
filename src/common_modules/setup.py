from setuptools import setup

package_name = 'common_modules'

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
            "offboard_control = common_modules.offboard_control:main",
            "local_position_listener = common_modules.local_position_listener:main",
            "target_position_publisher = common_modules.target_position_publisher:main",
            "uwb_anchor = common_modules.uwb_anchor:main"
        ],
    },
)
