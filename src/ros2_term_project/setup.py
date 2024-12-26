from setuptools import setup, find_packages

package_name = 'ros2_term_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 스크립트에 실행 권한 부여
        ('lib/' + package_name, ['scripts/ddpg_agent.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jewoo Kwak',
    maintainer_email='23512093@seoultech.ac.kr',
    description='ros2 term project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ddpg_agent = ros2_term_project.scripts.ddpg_agent:main',
        ],
    },
)