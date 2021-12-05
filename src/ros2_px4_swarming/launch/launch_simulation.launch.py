import datetime
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory, get_package_prefix

RECORD_ON = False


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
        '..',
        '..',
        '..',
        '..',
        'src',
        'ros2_px4_swarming',
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

    if numDrones > 0:
        # Launch node publishing numDrones
        numDronesNode = Node(
            package='ros2_px4_swarming',
            namespace='numAnchorsNode',
            executable='numAnchorsNode',
            name='numAnchorsNode',
            parameters=[
                params,
                {'N': numDrones}
            ]
        )
        ld.add_action(numDronesNode)

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

    for i in range(numDrones):
        # Launch drone
        anchorDrone = Node(
            package='ros2_px4_swarming',
            namespace='X500_' + str(i),
            executable='anchorDrone',
            name='X500_' + str(i),
            parameters=[
                params,
                {'N': numDrones}
            ]
        )
        ld.add_action(anchorDrone)

    if numDrones > 1:
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
            ]
        )
        ld.add_action(trackingVelocityCalculator)

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

        if RECORD_ON:
            # # Launch ros2bag
            # bagName = datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
            # cmdVector = ['ros2', 'bag', 'record', '-a', '-o', bagfiles + '/' + bagName]
            # ros2Record = ExecuteProcess(
            #     cmd=cmdVector,
            #     output='screen'
            # )
            # ld.add_action(ros2Record)

            # Launch topicsRecorder
            topicsRecorder = Node(
                package='ros2_px4_swarming',
                namespace='topicsRecorder',
                executable='topicsRecorder',
                name='topicsRecorder',
                parameters=[
                    params,
                    {'N': numDrones},
                    {'NUM_TARGET': numTarget}
                ]
            )
            ld.add_action(topicsRecorder)

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
        ]
    )
    ld.add_action(performanceAnalyzer)

    return ld
