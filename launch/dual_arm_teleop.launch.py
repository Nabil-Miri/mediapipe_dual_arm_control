import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch_param_builder import ParameterBuilder


def generate_launch_description():
    
    # Get config directory from this package
    config_dir = os.path.join(
        get_package_share_directory("mediapipe_dual_arm_control"), "config"
    )
    # Load robot description (URDF) using xacro
    urdf_path = os.path.join(config_dir, "robot", "panda.urdf.xacro")
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Load semantic description (SRDF)
    with open(os.path.join(config_dir, "robot", "panda.srdf"), 'r') as srdf_file:
        robot_description_semantic = {'robot_description_semantic': srdf_file.read()}

    # Compose moveit_config dict
    moveit_config = {}
    moveit_config.update(robot_description)
    moveit_config.update(robot_description_semantic)

    left_servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(os.path.join(config_dir, "pose_tracking", "pose_tracking_settings.yaml"))
        .yaml(os.path.join(config_dir, "pose_tracking", "dual_arm_left_config_pose_tracking.yaml"))
        .to_dict()
    }

    right_servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(os.path.join(config_dir, "pose_tracking", "pose_tracking_settings.yaml"))
        .yaml(os.path.join(config_dir, "pose_tracking", "dual_arm_right_config_pose_tracking.yaml"))
        .to_dict()
    }

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("mediapipe_dual_arm_control"),
        "config",
        "rviz",
        "dual_arm_teleop.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Left arm pose tracking node
    left_pose_tracking_node = Node(
        package="mediapipe_dual_arm_control",
        executable="dual_arm_servo_node",
        name="left_arm_pose_tracking",
        output="screen",
        parameters=[
            moveit_config,
            left_servo_params,
            {"arm_side": "left"}
        ],
    )

    # Right arm pose tracking node
    right_pose_tracking_node = Node(
        package="mediapipe_dual_arm_control",
        executable="dual_arm_servo_node", 
        name="right_arm_pose_tracking",
        output="screen",
        parameters=[
            moveit_config,
            right_servo_params,
            {"arm_side": "right"}
        ],
    )

    # # MediaPipe hand tracking coordinator node
    # mediapipe_coordinator = Node(
    #     package="moveit_servo",
    #     executable="mediapipe_dual_arm_coordinator",
    #     name="mediapipe_coordinator",
    #     output="screen",
    # )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("mediapipe_dual_arm_control"),
        "config",
        "robot",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
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

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    # Gripper controllers spawners
    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_panda_fingers_controller", "-c", "/controller_manager"],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_panda_fingers_controller", "-c", "/controller_manager"],
    )

    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[moveit_config],
    # )

    return LaunchDescription(
        [
            robot_state_publisher,
            static_tf,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            left_gripper_controller_spawner,
            right_gripper_controller_spawner,
            left_pose_tracking_node,
            right_pose_tracking_node,
            # mediapipe_coordinator,
            # move_group_node,
            rviz_node,
        ]
    )
