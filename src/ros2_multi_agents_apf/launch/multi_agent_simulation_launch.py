from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from time import gmtime, strftime
import os
import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_path = str(os.path.realpath(get_package_share_directory('ros2_artificial_potential_field')))
    # mission_file_name = package_path + "/mission/mission_multi_agent_2.yaml"
    mission_file_name = package_path + "/mission/mission_multi_agent_30.yaml"
    with open(mission_file_name, encoding='UTF-8') as f:
        mission = yaml.load(f, Loader=yaml.FullLoader)
    agents = mission['agents']
    number_of_agent = len(agents)

    # 런치 파라미터 선언
    alpha_arg = DeclareLaunchArgument('alpha', default_value='10.0', description='Alpha parameter')
    beta_arg = DeclareLaunchArgument('beta', default_value='5.0', description='Beta parameter')
    influence_distance_arg = DeclareLaunchArgument('influence_distance', default_value='3.0', description='Influence distance')

    ld = LaunchDescription()

    # 런치 아규먼트 추가
    ld.add_action(alpha_arg)
    ld.add_action(beta_arg)
    ld.add_action(influence_distance_arg)

    for agent_id in range(0, number_of_agent):
        namespace="cf" + str(agents[agent_id]["crazyflie_id"])

        traj_planner_node = Node(
            package="ros2_artificial_potential_field",
            namespace=namespace,
            executable="agent_node",
            output="screen",
            name="agent",
            parameters=[
                {"agent_id": agent_id,
                 "mission_file_name": mission_file_name},
                {"alpha": LaunchConfiguration("alpha")},
                {"beta": LaunchConfiguration("beta")},
                {"influence_distance": LaunchConfiguration("influence_distance")}
            ]
        )
        ld.add_action(traj_planner_node)

    rviz_path = package_path + '/config/rviz_config_sim.rviz'
    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
    )
    ld.add_action(rviz_node)

    return ld
