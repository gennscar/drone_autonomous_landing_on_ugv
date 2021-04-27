from setuptools import setup

package_name = 'control'

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
    maintainer='cosimo',
    maintainer_email='cosimocon@yahoo.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "hand_of_god_nav = control.hand_of_god_nav:main",
          "target_follower = control.target_follower:main",
          "offboard_control = control.offboard_control:main"
        ],
    },
)
