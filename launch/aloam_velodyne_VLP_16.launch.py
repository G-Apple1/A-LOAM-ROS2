from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 启动参数：是否打开 rviz
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RVIZ if true'
    )

    # 包路径
    package_name = 'aloam_velodyne_ros2'
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz_cfg',
        'aloam_velodyne_ros2.rviz'
    ])

    # ===================== 全局参数 =====================
    scan_line = 16
    mapping_skip_frame = 1 
    minimum_range = 1.0
    mapping_line_res =  0.2
    mapping_plane_res = 0.4

    # ===================== 三个 A-LOAM 核心节点 =====================
    ascanRegistration_node = Node(
        package='aloam_velodyne_ros2',
        executable='ascanRegistration',
        name='ascanRegistration',
        output='screen',
        parameters=[
            {'scan_line': scan_line},
            {'minimum_range': minimum_range}
        ]
    )

    alaserOdometry_node = Node(
        package='aloam_velodyne_ros2',
        executable='alaserOdometry',
        name='alaserOdometry',
        output='screen'
    )

    alaserMapping_node = Node(
        package='aloam_velodyne_ros2',
        executable='alaserMapping',
        name='alaserMapping',
        output='screen',
        parameters=[
            {'mapping_skip_frame': mapping_skip_frame},
            {'mapping_line_resolution': mapping_line_res},
            {'mapping_plane_resolution': mapping_plane_res}
        ]
    )


    transform_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_map',
        # arguments=['0', '0', '0', '1.570795', '0', '1.570795', 'map', 'camera_init'],
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
    )

    # ===================== RVIZ =====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        ascanRegistration_node,
        alaserOdometry_node,
        alaserMapping_node,
        rviz_node,
        transform_map
    ])