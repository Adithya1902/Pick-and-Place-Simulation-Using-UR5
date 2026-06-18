from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_ur5_gripper_moveit_config", package_name="my_ur5_gripper_moveit_config")
        .robot_description(file_path="config/ur5_robotiq.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    bt_node = Node(
        package="my_pick_and_place",
        executable="pick_and_place_bt_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}  
        ]
    )

    return LaunchDescription([bt_node])
