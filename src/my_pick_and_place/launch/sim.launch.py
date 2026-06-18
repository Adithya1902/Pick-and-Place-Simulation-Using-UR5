import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_my_pick_and_place = FindPackageShare('my_pick_and_place')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_moveit_config = FindPackageShare('my_ur5_gripper_moveit_config')
    world_file = PathJoinSubstitution([pkg_my_pick_and_place, 'world', 'pick_and_place.sdf'])
    xacro_file = PathJoinSubstitution([pkg_my_pick_and_place, 'urdf', 'ur5_with_gripper.urdf.xacro'])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    robot_description_content = Command(['xacro ', xacro_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_content, '-name', 'ur5_robotiq', '-allow_renaming', 'true', '-z', '0.0'],
        output='screen',
    )

    joint_state_broadcaster = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    arm_controller = Node(package="controller_manager", executable="spawner", arguments=["joint_trajectory_controller"])
    gripper_controller = Node(package="controller_manager", executable="spawner", arguments=["gripper_position_controller"])

    moveit_config = (
        MoveItConfigsBuilder("my_ur5_gripper_moveit_config", package_name="my_ur5_gripper_moveit_config")
        .robot_description(file_path="config/ur5_robotiq.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    rviz_config_file = PathJoinSubstitution([pkg_moveit_config, "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
    )
    
    robotiq_path = os.path.join(get_package_share_directory('robotiq_description'), '..', '..')
    set_gz_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', robotiq_path)

    return LaunchDescription([
        set_gz_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        move_group_node,
        rviz_node,
    ])
