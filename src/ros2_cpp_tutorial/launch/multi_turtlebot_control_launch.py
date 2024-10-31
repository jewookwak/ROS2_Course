from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    number_of_sim = 4

    ld = LaunchDescription()
    for sim_id in range(1, number_of_sim+1):
        simulator_node = Node(
            package='turtlesim',
            namespace= 'name'+str(sim_id),
            executable='turtlesim_node',
            name='sim'
        )
        ld.add_action(simulator_node)

        controller_node = Node(
            package= 'ros2_cpp_tutorial',
            namespace= 'name'+str(sim_id),
            executable= 'turtlebot_controller',
            name='controller'
        )
        ld.add_action(controller_node)
    return ld