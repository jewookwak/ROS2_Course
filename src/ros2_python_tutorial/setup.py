import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_python_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jewoo',
    maintainer_email='jewoo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = ros2_python_tutorial.python_node:main',
            'talker = ros2_python_tutorial.publisher_member_function:main',
            'listener = ros2_python_tutorial.subscriber_member_function:main',
            'double_talker = ros2_python_tutorial.p1_publisher:main',
            'double_listener = ros2_python_tutorial.p1_subscriber:main',
            'fast_agent = ros2_python_tutorial.p2_fast_agent:main',
            'slow_agent = ros2_python_tutorial.p2_slow_agent:main',
            'server = ros2_python_tutorial.service_member_function:main',
            'client = ros2_python_tutorial.client_member_function:main',
            'toggle_led_service = ros2_python_tutorial.p3_led_service:main',
            'toggle_led_client = ros2_python_tutorial.p3_led_client:main',
            'custom_msg_publisher = ros2_python_tutorial.custom_msg_publisher:main',
            'square = ros2_python_tutorial.square:main',
            'minimal_param_node = ros2_python_tutorial.python_parameters_node:main',
            'turtlebot_visualizer = ros2_python_tutorial.turtlebot_visualizer:main',
        ],
    },
)
