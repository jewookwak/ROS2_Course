import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 기존 cmd_publisher_node
    controller_node = Node(
        package='ros2_term_project',
        executable='cmd_publisher_node',
        name='cmd_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Python DDPG agent node
    ddpg_node = Node(
        package='ros2_term_project',
        executable='ddpg_agent.py',
        name='ddpg_agent',
        parameters=[{
                'is_training': True,  # or False
                'use_sim_time': use_sim_time
            }],
        output='screen',
    )

    # map_exploration_env node 추가
    exploration_node = Node(
        package='ros2_term_project',
        executable='map_exploration_node',
        name='map_exploration_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch Description 생성
    ld = LaunchDescription()
    
    # use_sim_time 파라미터 선언
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'))
    
    # 노드 추가
    ld.add_action(controller_node)
    ld.add_action(ddpg_node)
    ld.add_action(exploration_node)  # map_exploration_env 노드 추가
    
    return ld