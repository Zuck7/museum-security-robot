import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'museum_security_robot'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    gz_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(share, 'worlds', 'museum.sdf')
    urdf = os.path.join(share, 'urdf', 'security_bot.urdf')
    bridge_cfg = os.path.join(share, 'config', 'gz_bridge.yaml')

    # Read the URDF once at launch time.  robot_state_publisher needs the full
    # XML as a string parameter, and the ros_gz 'create' node then spawns the
    # model straight off the /robot_description topic - so the simulator and
    # the TF tree are guaranteed to describe the same robot.
    with open(urdf) as f:
        robot_description = f.read()

    # Spawn pose - must match spawn_pose in config/locations.json, which the
    # AI navigator feeds to AMCL as the initial pose.  z sits just above the
    # wheel radius so the robot settles onto the floor instead of intersecting.
    args = [
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-8.5'),
        DeclareLaunchArgument('z', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre2',
            description="use 'ogre' if you are on software rendering (ARM64 VM)"),
        DeclareLaunchArgument(
            'gz_verbosity', default_value='3'),
    ]

    gz_flags = ['-r -v ', LaunchConfiguration('gz_verbosity'),
                ' --render-engine ', LaunchConfiguration('render_engine'),
                ' ', world]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags,
                          'on_exit_shutdown': 'true'}.items(),
    )

    # Publishes the fixed transforms (base_link -> lidar_link, -> wheels,
    # -> caster) from the URDF, plus the moving wheel joints using
    # /joint_states from the bridge.  This is the base_link end of the
    # map -> odom -> base_link chain.
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'security_bot',
                   '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('yaw')],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
    )

    # Publishes odom -> base_link on /tf from the bridged /odom message.
    # The DiffDrive plugin could do this itself, but we keep it on the ROS
    # side so there is exactly one publisher for that TF edge.
    odom_tf = Node(
        package=PKG, executable='odom_tf_broadcaster', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [gazebo, rsp, spawn, bridge, odom_tf])
