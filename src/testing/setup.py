from setuptools import setup

package_name = 'testing'

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
            "uwb_pos_estimation = testing.uwb_pos_estimation:main",
            "positioning_error = testing.positioning_error:main",
            "video_streamer = testing.video_streamer:main"
        ],
    },
)
