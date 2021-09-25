from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    """
    ld.add_action(ExecuteProcess(
        cmd=['/home/gennscar/PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh', '-t', 'px4_sitl_rtps','-s', 'r1_rover:1,iris:1,', '-l','rtps'],
        output='screen',
    ))
    ld.add_action(ExecuteProcess(
        cmd=['micrortps_agent', '-t', 'UDP', '-r', '2020','-s', '2019', '-n', 'rover'],
        output='screen',
    ))
    ld.add_action(ExecuteProcess(
        cmd=['micrortps_agent', '-t', 'UDP', '-r', '2022','-s', '2021', '-n', 'drone'],
        output='screen',
    ))"""

    ld.add_action(ExecuteProcess(
        cmd=['/home/gennscar/PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh', '-t', 'px4_sitl_rtps','-s', 'iris:1,', '-l','rtps'],
        output='screen',
    ))
    ld.add_action(ExecuteProcess(
        cmd=['micrortps_agent', '-t', 'UDP', '-r', '2020','-s', '2019', '-n', 'drone'],
        output='screen',
    ))

    return ld