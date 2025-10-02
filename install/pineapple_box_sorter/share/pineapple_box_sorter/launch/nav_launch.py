from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('pineapple_box_sorter')
    
    # File paths
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'pa_rviz_nav2.rviz'])
    map_yaml = PathJoinSubstitution([pkg_share, 'map', 'pa_warehouse_map_02.data'])
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'pa_nav2_params.yaml'])
    
    # Launch arguments (optional - for flexibility)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_lidar',
        arguments=[
            '0', '0', '0',           # x y z
            '1', '0', '0', '0',      # qx qy qz qw 
            'virtual_hand_solo/base_link',
            'virtual_hand_solo/lidar_link'
        ]
    )

    # Map Server - Lifecycle Node
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_yaml,
             'use_sim_time': use_sim_time}
        ]
    )

    # Controller Server - Lifecycle Node
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Planner Server - Lifecycle Node
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Behavior Server - Lifecycle Node
    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # BT Navigator - Lifecycle Node
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Waypoint Follower - Lifecycle Node
    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Velocity Smoother - Lifecycle Node
    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # SLAM Toolbox Node
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # or 'lifelong_slam_toolbox_node'
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            nav2_params,
            {
             'use_sim_time': use_sim_time,
             'map_file_name': map_yaml,
             'map_start_pose': [0.0, 0.0, 0.0],  # Optional: set initial pose
             'use_sim_time': False,
             'mode': "localization",
             'map_start_pose': [0.0, 0.0, 0.0],
             'odom_frame': "virtual_hand_solo/odom",
             'base_frame': "virtual_hand_solo/base_link",
             'map_frame': "map",
             'scan_topic': "/virtual_hand_solo/scan",
             'resolution': 0.05,
             'max_laser_range': 10.0,
             'transform_publish_period': 0.05,
             'publish_map': True,
             'publish_pose': True,
             }
        ]
    )

    # Lifecycle Manager for SLAM (optional, if you want to manage SLAM node lifecycle)
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['slam_toolbox']}
        ]
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )    

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        
        # TF
        static_tf,

        # SLAM Stack
        slam_toolbox,
        lifecycle_manager_slam,

        # Navigation Stack
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
        
        # Visualization
        rviz,
    ])