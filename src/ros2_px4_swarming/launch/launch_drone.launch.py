import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    ld = LaunchDescription()

    # Environment variables
    ID = int(os.environ['DRONE_ID'])

    # Parameters file path
    params = os.path.join(
        get_package_share_directory('ros2_px4_swarming'),
        'parameters',
        'drone_params.yaml'
    )

    # Launch drone
    anchorDrone = Node(
        package='ros2_px4_swarming',
        namespace='X500_' + str(ID),
        executable='anchorDrone',
        name='X500_' + str(ID),
        parameters=[
            params,
            {'ID': ID}
        ],
        remappings=[
            ('/X500_' + str(ID) + '/uwb_sensor_' + str(ID), '/uwb_sensor_' + str(ID)),
            ('/X500_' + str(ID) + '/trackingVelocityCalculator/trackingVelocity', '/trackingVelocityCalculator/trackingVelocity'),
            ('/X500_' + str(ID) + '/numAnchorsNode/N', '/numAnchorsNode/N'),
            ('/X500_' + str(ID) + '/X500_0/readyForSwarming', '/X500_0/readyForSwarming'),
            ('/X500_' + str(ID) + '/X500_1/readyForSwarming', '/X500_1/readyForSwarming'),
            ('/X500_' + str(ID) + '/X500_2/readyForSwarming', '/X500_2/readyForSwarming'),
            ('/X500_' + str(ID) + '/X500_3/readyForSwarming', '/X500_3/readyForSwarming'),
            ('/X500_' + str(ID) + '/X500_4/readyForSwarming', '/X500_4/readyForSwarming'),
            ('/X500_' + str(ID) + '/X500_5/readyForSwarming', '/X500_5/readyForSwarming'),
            ('/X500_' + str(ID) + '/X500_0/readyForTakeoff', '/X500_0/readyForTakeoff'),
            ('/X500_' + str(ID) + '/X500_1/readyForTakeoff', '/X500_1/readyForTakeoff'),
            ('/X500_' + str(ID) + '/X500_2/readyForTakeoff', '/X500_2/readyForTakeoff'),
            ('/X500_' + str(ID) + '/X500_3/readyForTakeoff', '/X500_3/readyForTakeoff'),
            ('/X500_' + str(ID) + '/X500_4/readyForTakeoff', '/X500_4/readyForTakeoff'),
            ('/X500_' + str(ID) + '/X500_5/readyForTakeoff', '/X500_5/readyForTakeoff')
        ]
    )
    ld.add_action(anchorDrone)

    return ld
