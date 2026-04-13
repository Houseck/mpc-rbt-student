import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    # student_pkg_dir = get_package_share_directory('mpc_rbt_student')
    solution_pkg_dir = get_package_share_directory('mpc_rbt_solution')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    # 2. Definice nody pro Warehouse Manager (Skladník)
    warehouse_manager = Node(
        package='mpc_rbt_solution',
        executable='warehouse_manager',
        name='warehouse_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Definice nody pro BT Server (Mozek)
    bt_server = Node(
        package='mpc_rbt_solution',
        executable='bt_server',
        name='bt_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            os.path.join(solution_pkg_dir, 'config', 'bt_server.yaml')
        ]
    )

    
    return LaunchDescription([
        Node(
            package='mpc_rbt_student',
            executable='localization',
            name='localization_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        Node(
            package='mpc_rbt_student',
            executable='planning_node', 
            name='planning_node',
            output='screen'
        ),
        Node(
            package='mpc_rbt_student',
            executable='motion_control_node', 
            name='motion_control_node',
            output='screen'
        ),
        warehouse_manager,
        bt_server
    ])
