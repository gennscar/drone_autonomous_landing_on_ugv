import datetime
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    ld = LaunchDescription()

    # Numbers of vehicles to spawn
    numTarget = int(os.environ['NUM_TARGET'])
    if numTarget > 0:
        numTarget = 1
    numDrones = int(os.environ['NUM_DRONES'])

    # Parameters file path
    params = os.path.join(
        get_package_share_directory('ros2_px4_swarming'),
        'parameters',
        'params.yaml'
    )

    # Bag files path
    bagfiles = os.path.join(
        get_package_prefix('ros2_px4_swarming'),
        '..',
        '..',
        'src',
        'ros2_px4_swarming',
        'bagfiles'
    )

    if numTarget == 1:
        # Launch target
        targetRover = Node(
            package='ros2_px4_swarming',
            namespace='targetRover',
            executable='targetRover',
            name='targetRover',
            parameters=[params]
        )
        ld.add_action(targetRover)

    if numDrones > 1:
        # Launch unitVectorsCalculator
        unitVectorsCalculator = Node(
            package='ros2_px4_swarming',
            namespace='unitVectorsCalculator',
            executable='unitVectorsCalculator',
            name='unitVectorsCalculator',
            parameters=[
                params,
                {'N': numDrones}
            ],
            remappings=[
                ('/unitVectorsCalculator/X500_0/VehicleGlobalPosition_PubSubTopic', '/X500_0/VehicleGlobalPosition_PubSubTopic'),
                ('/unitVectorsCalculator/X500_1/VehicleGlobalPosition_PubSubTopic', '/X500_1/VehicleGlobalPosition_PubSubTopic'),
                ('/unitVectorsCalculator/X500_2/VehicleGlobalPosition_PubSubTopic', '/X500_2/VehicleGlobalPosition_PubSubTopic'),
                ('/unitVectorsCalculator/X500_3/VehicleGlobalPosition_PubSubTopic', '/X500_3/VehicleGlobalPosition_PubSubTopic'),
                ('/unitVectorsCalculator/X500_4/VehicleGlobalPosition_PubSubTopic', '/X500_4/VehicleGlobalPosition_PubSubTopic'),
                ('/unitVectorsCalculator/X500_5/VehicleGlobalPosition_PubSubTopic', '/X500_5/VehicleGlobalPosition_PubSubTopic')
            ]
        )
        ld.add_action(unitVectorsCalculator)

        # Launch trackingVelocityCalculator
        trackingVelocityCalculator = Node(
            package='ros2_px4_swarming',
            namespace='trackingVelocityCalculator',
            executable='trackingVelocityCalculator',
            name='trackingVelocityCalculator',
            parameters=[
                params,
                {'N': numDrones},
                {'NUM_TARGET': numTarget}
            ],
            remappings=[
                ('/trackingVelocityCalculator/uwb_sensor_200', '/uwb_sensor_200'),
                ('/trackingVelocityCalculator/unitVectorsCalculator/unitVectors', '/unitVectorsCalculator/unitVectors'),
                ('/trackingVelocityCalculator/X500_0/VehicleGlobalPosition_PubSubTopic', '/X500_0/VehicleGlobalPosition_PubSubTopic'),
                ('/trackingVelocityCalculator/X500_1/VehicleGlobalPosition_PubSubTopic', '/X500_1/VehicleGlobalPosition_PubSubTopic'),
                ('/trackingVelocityCalculator/X500_2/VehicleGlobalPosition_PubSubTopic', '/X500_2/VehicleGlobalPosition_PubSubTopic'),
                ('/trackingVelocityCalculator/X500_3/VehicleGlobalPosition_PubSubTopic', '/X500_3/VehicleGlobalPosition_PubSubTopic'),
                ('/trackingVelocityCalculator/X500_4/VehicleGlobalPosition_PubSubTopic', '/X500_4/VehicleGlobalPosition_PubSubTopic'),
                ('/trackingVelocityCalculator/X500_5/VehicleGlobalPosition_PubSubTopic', '/X500_5/VehicleGlobalPosition_PubSubTopic'),
                ('/trackingVelocityCalculator/X500_0/readyForSwarming', '/X500_0/readyForSwarming'),
                ('/trackingVelocityCalculator/X500_1/readyForSwarming', '/X500_1/readyForSwarming'),
                ('/trackingVelocityCalculator/X500_2/readyForSwarming', '/X500_2/readyForSwarming'),
                ('/trackingVelocityCalculator/X500_3/readyForSwarming', '/X500_3/readyForSwarming'),
                ('/trackingVelocityCalculator/X500_4/readyForSwarming', '/X500_4/readyForSwarming'),
                ('/trackingVelocityCalculator/X500_5/readyForSwarming', '/X500_5/readyForSwarming')
            ]
        )
        ld.add_action(trackingVelocityCalculator)

        # Launch performanceAnalyzer
        performanceAnalyzer = Node(
            package='ros2_px4_swarming',
            namespace='performanceAnalyzer',
            executable='performanceAnalyzer',
            name='performanceAnalyzer',
            parameters=[
                params,
                {'N': numDrones},
                {'NUM_TARGET': numTarget}
            ],
            remappings=[
                ('/performanceAnalyzer/uwb_sensor_200', '/uwb_sensor_200'),
                ('/performanceAnalyzer/targetRover/GroundTruth/odom', '/targetRover/GroundTruth/odom'),
                ('/performanceAnalyzer/X500_0/GroundTruth/odom', '/X500_0/GroundTruth/odom'),
                ('/performanceAnalyzer/X500_1/GroundTruth/odom', '/X500_1/GroundTruth/odom'),
                ('/performanceAnalyzer/X500_2/GroundTruth/odom', '/X500_2/GroundTruth/odom'),
                ('/performanceAnalyzer/X500_3/GroundTruth/odom', '/X500_3/GroundTruth/odom'),
                ('/performanceAnalyzer/X500_4/GroundTruth/odom', '/X500_4/GroundTruth/odom'),
                ('/performanceAnalyzer/X500_5/GroundTruth/odom', '/X500_5/GroundTruth/odom')
            ]
        )
        ld.add_action(performanceAnalyzer)

        # # Launch rqt_plot
        # cmdVector = ['ros2', 'run', 'rqt_plot', 'rqt_plot']
        # # cmdVector.append('/performanceAnalyzer/trackingError/distance')
        # # for i in range(int(numDrones * (numDrones - 1) / 2)):
        # #     cmdVector.append('/performanceAnalyzer/interAnchorsDistances/data[' + str(i) + ']/distance')
        # cmdVector.append('/trackingVelocityCalculator/trackingVelocity')
        # rqtPlot = ExecuteProcess(
        #     cmd=cmdVector,
        #     output='screen'
        # )
        # ld.add_action(rqtPlot)

        # # Launch ros2bag
        # bagName = datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
        # cmdVector = ['ros2', 'bag', 'record', '-a', '-o', bagfiles + '/' + bagName]
        # ros2Record = ExecuteProcess(
        #     cmd=cmdVector,
        #     output='screen'
        # )
        # ld.add_action(ros2Record)

    return ld
