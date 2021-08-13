
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource


def generate_launch_description():


    pkg_share       = get_package_share_directory('dots_example_controller')

    controller_option = LaunchConfiguration('controllerOptions')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    robot_name      = LaunchConfiguration('robot_name')    

    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_robot_name      = DeclareLaunchArgument('robot_name', default_value='robot_deadbeef')


    setup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'basic_cam.launch.py')),
    )

    controller_cmd = Node(
        package     = 'dots_example_controller',
        executable  = controller_option,
        namespace   = robot_name,
        output      = 'screen',
        parameters  = [ {'use_sim_time' : use_sim_time}]
    )


    # Build the launch description
    ld = LaunchDescription()

    bag_name = os.environ.get('ROBOTID')

    ld.add_entity(ExecuteProcess(

        cmd=['ros2', 'bag', 'record',

             '--compression-mode', 'file',

             '--compression-format', 'zstd',

             # '--qos-profile-overrides-path', 'override.yaml',

             '-o', '/storage/%s' % bag_name,

             '/robot_tasks'],

        output='screen'

    ))

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_name)
    ld.add_action(setup_cmd)
    ld.add_action(controller_cmd)

    return ld