from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    sim_share = get_package_share_directory('drobot_simulation')
    desc_share = get_package_share_directory('drobot_description')
    world_path = os.path.join(desc_share, 'worlds', 'empty.sdf')
    bridge_yaml = os.path.join(sim_share, 'config', 'bridge.yaml')
    xacro_path = os.path.join(desc_share, 'urdf', 'drobot.urdf.xacro')

    # IMPORTANT: model://drobot_description/... needs parent of desc_share
    desc_share_parent = os.path.dirname(desc_share)
    gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_path = f"{desc_share_parent}:{gz_path}" if gz_path else desc_share_parent
    

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

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-name', 'drobot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0',
            '-z', '0.05'   # <-- HEIGHT (meters)
        ],
    )


    return LaunchDescription([gazebo, bridge, rsp, spawn])
