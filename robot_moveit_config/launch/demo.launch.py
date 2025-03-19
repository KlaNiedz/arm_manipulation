from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("six_dof_arm", package_name="robot_moveit_config").to_moveit_configs()
    # return generate_demo_launch(moveit_config)

    moveit_config = (
        MoveItConfigsBuilder("six_dof_arm", package_name="robot_moveit_config")
        .robot_description(file_path="config/six_dof_arm.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)