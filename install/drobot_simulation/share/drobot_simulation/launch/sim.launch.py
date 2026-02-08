from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
import os


def generate_launch_description():
    sim_share = get_package_share_directory('drobot_simulation')
    desc_share = get_package_share_directory('drobot_description')
    default_world = os.path.join(desc_share, 'worlds', 'basic.sdf')
    world_path = LaunchConfiguration('world')
    bridge_yaml = os.path.join(sim_share, 'config', 'bridge.yaml')
    xacro_path = os.path.join(desc_share, 'urdf', 'drobot.urdf.xacro')
    ekf_yaml = os.path.join(sim_share, 'config', 'ekf.yaml')

    # IMPORTANT: model://drobot_description/... needs parent of desc_share
    desc_share_parent = os.path.dirname(desc_share)
    gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_path = f"{desc_share_parent}:{gz_path}" if gz_path else desc_share_parent

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to an SDF world file.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with nav visualization config.',
    )
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch teleop_twist_keyboard in a new terminal.',
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'QT_XCB_GL_INTEGRATION': 'x11',
            'GZ_RENDER_ENGINE': 'ogre2',
            'GZ_SIM_RESOURCE_PATH': gz_path,
        }
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_yaml}],
    )

    robot_description = ParameterValue(
        Command([f'xacro {xacro_path}']),
        value_type=str
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(sim_share, 'config', 'nav2_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-lc',
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
            '--ros-args --remap cmd_vel:=/cmd_vel'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'drobot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0',
            '-z', '0.05'   # <-- HEIGHT (meters)
        ],
    )


    return LaunchDescription([world_arg, use_rviz_arg, use_teleop_arg, gazebo, bridge, ekf, rsp, rviz, teleop, spawn])
