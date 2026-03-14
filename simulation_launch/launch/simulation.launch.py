# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
 
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
 
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():
    # =============================================================================
    # Package paths
    # =============================================================================
    fra2mo_pkg_path = get_package_share_directory('ros2_fra2mo')
    simulation_pkg_path = get_package_share_directory('simulation_launch')
    models_path = os.path.join(fra2mo_pkg_path, 'models')
    world_file = os.path.join(fra2mo_pkg_path, "worlds", "leonardo_inspection_world.sdf")
    
    # =============================================================================
    # Declare arguments
    # =============================================================================
    # IIWA Robot arguments
    robot_args = [
        ('runtime_config_package', 'iiwa_description', 'Package with controller configuration'),
        ('controllers_file', 'iiwa_controllers.yaml', 'YAML file with controllers configuration'),
        ('description_package', 'iiwa_description', 'Package with robot URDF/xacro files'),
        ('description_file', 'iiwa.config.xacro', 'URDF/XACRO description file'),
        ('prefix', '""', 'Prefix for joint names (multi-robot setup)'),
        ('namespace', 'iiwa', 'Namespace for launched nodes (multi-robot setup)'),
    ]
    
    # Simulation arguments
    sim_args = [
        ('use_sim', 'true', 'Start robot in Gazebo simulation'),
        ('use_fake_hardware', 'true', 'Use fake hardware mirroring'),
        ('gz_args', [world_file, ' -r'], 'Arguments for gz_sim'),
    ]
    
    # Planning and controller arguments
    control_args = [
        ('use_planning', 'false', 'Start with Moveit2 planning'),
        ('use_servoing', 'false', 'Start with Moveit2 servoing'),
        ('robot_controller', 'iiwa_arm_controller', 'Robot controller to start'),
        ('command_interface', 'position', 'Command interface [position|velocity|effort]'),
    ]
    
    # Configuration and visualization arguments
    config_args = [
        ('start_rviz', 'false', 'Start RViz2 automatically'),
        ('robot_port', '30200', 'Robot port of FRI interface'),
        ('initial_positions_file', 'initial_positions.yaml', 'Robot initial positions config'),
        ('base_frame_file', 'base_frame.yaml', 'Robot base frame wrt World config'),
    ]
    
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in robot_args + sim_args + control_args + config_args
    ]
 
    # =============================================================================
    # Initialize Arguments
    # =============================================================================
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    robot_port = LaunchConfiguration('robot_port')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')
 
    # =============================================================================
    # Robot Descriptions
    # =============================================================================
    # IIWA Robot
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare(description_package), 'config', description_file]), ' ',
        'prefix:=', prefix, ' ',
        'use_sim:=', use_sim, ' ',
        'use_fake_hardware:=', use_fake_hardware, ' ',
        'robot_port:=', robot_port, ' ',
        'initial_positions_file:=', initial_positions_file, ' ',
        'command_interface:=', command_interface, ' ',
        'base_frame_file:=', base_frame_file, ' ',
        'description_package:=', description_package, ' ',
        'runtime_config_package:=', runtime_config_package, ' ',
        'controllers_file:=', controllers_file, ' ',
        'namespace:=', namespace,
    ])
    robot_description = {'robot_description': robot_description_content}
 
    # Fra2mo Robot
    fra2mo_xacro = os.path.join(fra2mo_pkg_path, "urdf", "fra2mo.urdf.xacro")
    fra2mo_description = {
        "robot_description": ParameterValue(Command(['xacro ', fra2mo_xacro]), value_type=str)
    }
 
    # =============================================================================
    # Configuration files
    # =============================================================================
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), 'config', controllers_file
    ])


    rviz_config_file = os.path.join(simulation_pkg_path, 'rviz_conf', 'fra2mo_conf.rviz')
 
    # =============================================================================
    # Launch files
    # =============================================================================
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'), '/launch/iiwa_planning.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'start_rviz': start_rviz,
            'base_frame_file': base_frame_file,
            'namespace': namespace,
            'use_sim': use_sim,
        }.items(),
        condition=IfCondition(use_planning),
    )
 
    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'), '/launch/iiwa_servoing.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'base_frame_file': base_frame_file,
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_servoing),
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
        condition=IfCondition(use_sim),
    )
 
    # =============================================================================
    # Nodes
    # =============================================================================
    # IIWA Nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description],
        condition=UnlessCondition(OrSubstitution(use_planning, use_sim)),
    )
 
    iiwa_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description, {"use_sim_time": True},],
        #remappings=[ ("/tf", "tf"), ("/tf_static", "tf_static"),]
    )

    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',\
        namespace=namespace,
        output='screen',
        arguments=[
            '-topic', '/iiwa/robot_description',
            '-name', 'iiwa',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.10',
        ],
        condition=IfCondition(use_sim),
    )
 
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', [namespace, '/controller_manager'],
                   #'-t','joint_state_broadcaster/JointStateBroadcaster'
                ]
    )
 
    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager', [namespace, '/controller_manager']],
        condition=UnlessCondition(use_sim),
    )
 
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller,
                   '--controller-manager', [namespace, '/controller_manager'],
                   #'-t','position_controllers/JointGroupPositionController'
                   ],
    )

    fra2mo_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros2_fra2mo'), 'launch', 'gazebo_fra2mo.launch.py'])
        ),
        #launch_arguments={'use_sim_time': 'true'}.items(),
    )
    
 
    # Fra2mo Nodes
    fra2mo_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="fra2mo",
        output="screen",
        parameters=[
            fra2mo_description,
            {"use_sim_time": True},
            #{'frame_prefix': 'fra2mo/'},
        ],
        #remappings=[("/tf", "tf"), ("/tf_static", "tf_static"),]
    )
 
    spawn_fra2mo = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/fra2mo/robot_description",
            "-name", "fra2mo",
            "-allow_renaming", "true",
            "-x", "0.0", "-y", "0.0", "-z", "0.10",
        ]
    )
 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
       arguments=[#'/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   #'/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   #'/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   #'/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   #'/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/iiwa/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/iiwa/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',],
                    #'--ros-args',
                    #'-r', '/camera:=/videocamera',],
        output='screen'
    )

    fra2mo_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='fra2mo',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}, {'frame_prefix': 'fra2mo/'}],
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace="fra2mo"
    )
 
    # =============================================================================
    # Event Handlers
    # =============================================================================
    delay_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )
 
    delay_joint_state_after_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )
 
    delay_rviz_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )
 
    delay_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
 
    # =============================================================================
    # Environment Variables
    # =============================================================================
    env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    # =============================================================================
    # Launch Description
    # =============================================================================
    nodes = [
        gazebo,

        control_node,
        iiwa_planning_launch,
        iiwa_servoing_launch,
        spawn_iiwa,
        iiwa_state_publisher,
        delay_joint_state_after_control,
        delay_joint_state_after_spawn,
        external_torque_broadcaster_spawner,
        delay_controller_after_joint_state,
        fra2mo_gazebo_launch,
        #fra2mo_state_publisher,
        #spawn_fra2mo,
        #odom_tf,
        #ign_clock_bridge,
        #fra2mo_rviz_node,
        bridge
    ]
    
    return LaunchDescription([env_var] + declared_arguments + nodes)