import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'dual_franka_description'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'dual_panda.urdf.xacro')
    
    # Xacro 변환
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     output='screen'
        # ),
        # 시뮬레이터가 켜지기 전까지 기본값(0)을 계속 쏴주는 노드
    #     Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     # 시뮬레이터와 토픽이 겹치지 않게 하려면 나중에 설정을 바꾸겠지만, 
    #     # 지금은 시뮬레이터를 켜면 이 노드를 Ctrl+C로 끄는 방식으로 충분합니다.
    # ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])