import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def servo_node_launch_description(arm_prefix, moveit_config):
    servo_yaml = load_yaml("ala_moveit_config", "config/servo_config.yaml")
    frame_config = {
        "move_group_name": f"{arm_prefix}arm",
        "planning_frame": f"{arm_prefix}base_link",
        # "planning_frame": "platform_link",
        "ee_frame_name": f"{arm_prefix}tool",
        "robot_link_command_frame": f"{arm_prefix}base_link",
        # "robot_link_command_frame": "platform_link",
        "command_out_topic": f"{arm_prefix}arm_controller/commands",
    }
    servo_yaml = {**servo_yaml, **frame_config}
    servo_params = {"moveit_servo": servo_yaml}

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{arm_prefix}arm_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{arm_prefix}gripper_controller", "-c", "/controller_manager"],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name=f"{arm_prefix}servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return [
        arm_controller_spawner,
        gripper_controller_spawner,
        servo_node,
    ]

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ala")
        .robot_description(file_path="config/ala.urdf.xacro")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("ala_moveit_config"),
        "config",
        "ros2_controllers_servo.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    joystick_to_twist_node = Node(
        package="ala_moveit_config",
        executable="joystick_control.py",
        name="joystick_to_twist_node",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )


    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            *servo_node_launch_description("left_", moveit_config),
            *servo_node_launch_description("right_", moveit_config),
            robot_state_publisher,
            # joystick_to_twist_node,
            # joy_node,
        ]
    )