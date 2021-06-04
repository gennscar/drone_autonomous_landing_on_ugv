from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import Shutdown

ukf_params = [
    [{'deltaT': 5e-3}, {'R_uwb': 1e-3}, {'R_px4': 0.25}, {'Q': 1e-3}],
    [{'deltaT': 5e-3}, {'R_uwb': 1e-3}, {'R_px4': 0.25}, {'Q': 1e-3}],
    [{'deltaT': 5e-3}, {'R_uwb': 1e-3}, {'R_px4': 0.25}, {'Q': 1e-3}],
    [{'deltaT': 5e-3}, {'R_uwb': 1e-3}, {'R_px4': 0.25}, {'Q': 1e-3}],
]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
                {"sensor_id": "0"},
                {"method": "LS"}
        ]
    ))

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='px4_positioning',
        namespace='PX4_estimator'
    ))

    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='positioning_error',
        namespace='positioning_error'
    ))

    for i, param in enumerate(ukf_params):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='ukf_positioning',
            namespace='UKF_estimator_' + str(i),
            parameters=param
        ))

    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 'data_records/simulated_uwb_gazebo'],
        output='screen',
        on_exit=Shutdown()
    ))

    return ld
