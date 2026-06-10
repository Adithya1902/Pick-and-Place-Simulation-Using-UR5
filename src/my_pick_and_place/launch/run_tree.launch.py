from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Gather the MoveIt configuration parameters
    moveit_config = (
        MoveItConfigsBuilder("my_ur5_gripper_moveit_config", package_name="my_ur5_gripper_moveit_config")
        .robot_description(file_path="config/ur5_robotiq.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. Wrap your C++ executable and sync the clocks
    bt_node = Node(
        package="my_pick_and_place",
        executable="pick_and_place_bt_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}  # <-- This tells the Brain to use Gazebo's clock
        ]
    )

    return LaunchDescription([bt_node])