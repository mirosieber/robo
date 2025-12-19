from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths
    pkg_share = get_package_share_directory('robo_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robo.urdf')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robo.rviz')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Controller manager node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_file
            ],
            output='screen'
        ),
        
        # Spawn joint state broadcaster
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        ),
        
        # Spawn drive controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'drive_controller'],
            output='screen'
        ),
        
        # Spawn steer controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'steer_controller'],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
