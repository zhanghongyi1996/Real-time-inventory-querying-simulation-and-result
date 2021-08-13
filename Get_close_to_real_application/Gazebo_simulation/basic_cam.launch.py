
import os
import glob
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete, OnProcessExit

'''
This launch file starts one robots support nodes. When running on the real robot,
this consists of just the robot state publisher, the EKF localisation filter, and
the TF relay. The TF relay allows each robot to have its own TF tree, from which 
transforms are echoed into the global /tf and /tf_static topics with frame names prefixed.

When running in simulation, denoted by the use_sim_time launch parameter, additional 
nodes are started to spawn the robot model into the simulator.

It is assumed that the simulator has already been started with gazebo.launch.py, or
that another node has been launch that provides the URDF on the topic /robot_description.
'''




def generate_launch_description():
    
    # Ensure no temporary URDF files around
    for f in glob.glob('/tmp/*.urdf'):
        try:
            os.remove(f)
        except OSError:
            print('Could not remove %s' % f)
    
    pkg_share           = get_package_share_directory('dots_example_controller')
    dots_sim_share      = get_package_share_directory('dots_sim')
    dots_vision_share   = get_package_share_directory('dots_vision')

    use_sim_time    = LaunchConfiguration('use_sim_time')
    robot_name      = LaunchConfiguration('robot_name')    
    robot_pose      = LaunchConfiguration('robot_pose')

    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_robot_name      = DeclareLaunchArgument('robot_name', default_value='robot_deadbeef')
    declare_robot_pose      = DeclareLaunchArgument('robot_pose', default_value='0,0,0')

    remappings = [('/tf', 'tf'),('/tf_static', 'tf_static')]


    #----------------------------------------------------------------------
    # This section is only started for simulation
    spawner_cmd = Node(
        condition   = IfCondition(use_sim_time),
        name        = PythonExpression(['"spawner_', robot_name, '"']),
        package     = 'gazebo_ros',
        executable  = 'spawn_entity.py',
        output      = 'screen',
        arguments   = [ '-topic', PythonExpression(['"/', robot_name, '/robot_description"']),
                        '-spawn_service_timeout', '30',
                        '-robot_namespace', PythonExpression(['"', robot_name, '"']),
                        PythonExpression(['"-x %f" % float("', robot_pose, '".split(",")[0])']),  
                        PythonExpression(['"-y %f" % float("', robot_pose, '".split(",")[1])']),  
                        PythonExpression(['"-Y %f" % float("', robot_pose, '".split(",")[2])']),  
                        '-z 0.0', 
                        '-entity', PythonExpression(['"', robot_name, '"'])],
    )

    # Publish a static transform so rviz doesn't complain about missing frame 'odom' before
    # simulator started
    init_transform_cmd = Node(
        condition   = IfCondition(use_sim_time),
        package     = 'tf2_ros',
        executable  = 'static_transform_publisher',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = remappings,
        arguments   = [ '0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    # When running a simulation, this node provides the topics normally
    # provided by the hardware interface node
    fake_node_cmd = Node(
        condition   = IfCondition(use_sim_time),
        package     = 'dots_sim_support',
        executable  = 'fake_dots_hardware',
        namespace   = robot_name,
        output      = 'screen',
        parameters  = [{'use_sim_time': use_sim_time}]
    )

    # Start robot_state_publisher
    # Because we cannot access the urdf file directly, potentially running in a different
    # container to the simulator which owns the robot model, and because robot_state_publisher
    # no longer can take a model from a topic (https://github.com/ros/robot_state_publisher/issues/144),
    # we use a helper node to take the published model topic and optionally prefix the urdf
    # and write it to a temporary local file.
    get_robot_urdf_cmd = Node(
        condition   = IfCondition(use_sim_time),
        package     = 'urdf_prefix',
        executable  = 'urdf_prefix',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = [('in_robot_description', '/robot_description')],
        parameters  = [{'filename' : robot_name,
                        'prefix' : PythonExpression(['"', robot_name, '_"'])
                        }],
    )

    robot_state_publisher_cmd = ExecuteProcess(
        condition   = IfCondition(use_sim_time),
        cwd         = os.path.join(dots_sim_share, 'launch'),
        cmd         = ['./rsp_helper.sh', PythonExpression(['"/tmp/', robot_name, '.urdf"']),
                        '--ros-args', 
                        '-p', 'use_sim_time:=True',
                        '-p', 'use_tf_static:=False',
                        '-r', PythonExpression(['"__ns:=/', robot_name, '"']),
                        '-r', PythonExpression(['"/tf:=tf"']),
                        '-r', PythonExpression(['"/tf_static:=tf_static"']),
                        ],
        output      = 'screen'
    )


    #--------------------------------------------------------------
    # These nodes are always started, for both simulation and real
    #--------------------------------------------------------------


    cam0_processing_cmd = Node(
        package     = 'dots_vision',
        executable  = 'vision',
        name        = 'cam0_vision',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = remappings + [('img_in', 'cam0/image'), 
                                    ('img_out', 'cam0_out'), 
                                    ('tags_out', 'cam0_tags')],
        parameters  = [{'use_sim_time'  : use_sim_time},
                       {'cam_name'      : 'cam0'},
                       {'frame_prefix'  : PythonExpression(['"', robot_name, '_"'])}]
    )

    cam1_processing_cmd = Node(
        package     = 'dots_vision',
        executable  = 'vision',
        name        = 'cam1_vision',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = remappings + [('img_in', 'cam1/image'), 
                                    ('img_out', 'cam1_out'), 
                                    ('tags_out', 'cam1_tags')],
        parameters  = [{'use_sim_time'  : use_sim_time},
                       {'cam_name'      : 'cam1'},
                       {'frame_prefix'  : PythonExpression(['"', robot_name, '_"'])}]
    )

    cam2_processing_cmd = Node(
        package     = 'dots_vision',
        executable  = 'vision',
        name        = 'cam2_vision',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = remappings + [('img_in', 'cam2/image'), 
                                    ('img_out', 'cam2_out'), 
                                    ('tags_out', 'cam2_tags')],
        parameters  = [{'use_sim_time'  : use_sim_time},
                       {'cam_name'      : 'cam2'},
                       {'frame_prefix'  : PythonExpression(['"', robot_name, '_"'])}]
    )

    cam3_processing_cmd = Node(
        package     = 'dots_vision',
        executable  = 'vision',
        name        = 'cam3_vision',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = remappings + [('img_in', 'cam3/image'), 
                                    ('img_out', 'cam3_out'), 
                                    ('tags_out', 'cam3_tags')],
        parameters  = [{'use_sim_time'  : use_sim_time},
                       {'cam_name'      : 'cam3'},
                       {'frame_prefix'  : PythonExpression(['"', robot_name, '_"'])}]
    )

    cam4_processing_cmd = Node(
        package     = 'dots_vision',
        executable  = 'vision',
        name        = 'cam4_vision',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = remappings + [('img_in', 'cam4/image'), 
                                    ('img_out', 'cam4_out'), 
                                    ('tags_out', 'cam4_tags')],
        parameters  = [{'use_sim_time'  : use_sim_time},
                       {'cam_name'      : 'cam4'},
                       {'marker_map'    : PythonExpression(['"', dots_vision_share, '/params/mm49_out.yml"'])},
                       {'frame_prefix'  : PythonExpression(['"', robot_name, '_"'])}]
    )

    # Start tf_relay. This listens on the local tf tree and republishes transforms on the global tree 
    # possibly with a prefix added.
    tf2_relay_cmd = Node(
        package     = 'dots_tf_tools',
        executable  = 'tf2_relay',
        namespace   = robot_name,
        output      = 'screen',
        remappings  = [ ('out/tf',          '/tf'), 
                        ('out/tf_static',   '/tf_static'),
                        ('in/tf',           'tf'),
                        ('in/tf_static',    'tf_static')],
        parameters  = [ #{'prefix' : PythonExpression(['"', robot_name, '_"'])},
                        {'exclude_frames' : ['map', 'odom']}]
    )


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_robot_pose)
    ld.add_action(spawner_cmd)
    ld.add_action(init_transform_cmd)
    #ld.add_action(map_to_odom_cmd)
    ld.add_action(fake_node_cmd)
    ld.add_action(get_robot_urdf_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(cam0_processing_cmd)
    ld.add_action(cam1_processing_cmd)
    ld.add_action(cam2_processing_cmd)
    ld.add_action(cam3_processing_cmd)
    ld.add_action(cam4_processing_cmd)
    ld.add_action(tf2_relay_cmd)



    return ld
    

