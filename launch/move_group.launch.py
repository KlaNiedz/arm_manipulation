from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("six_dof_arm", package_name="robot_moveit_config").to_moveit_configs()
    if not moveit_config.move_group_capabilities:
        moveit_config.move_group_capabilities = {"capabilities": []}
    return generate_move_group_launch(moveit_config)
