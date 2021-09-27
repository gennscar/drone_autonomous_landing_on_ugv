from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()



    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='drone_rover_positioning_error',
        namespace='drone_rover_positioning_error',
        parameters=[
            {"vehicle_namespace": "/rover"},
            {"sensor_id": "Iris"}
        ]
        ))

    
    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='rover_yaw_error',
        namespace='rover_yaw_error',
        parameters=[
            {"vehicle_namespace": "/rover"}
        ]))
    
    return ld