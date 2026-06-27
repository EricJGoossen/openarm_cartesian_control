import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

XACRO_PATH = os.path.join(
    get_package_share_directory("openarm_description"),
    "assets", "robot", "openarm_v1.0", "urdf", "openarm_v10.urdf.xacro"
)

LAUNCH_DELAY_SECONDS = 1.0


def generate_robot_description(context: LaunchContext,
                                right_can_interface,
                                left_can_interface) -> str:
    return xacro.process_file(
        XACRO_PATH,
        mappings={
            "arm_type": "openarm_v1.0",
            "bimanual": "true",
            "use_fake_hardware": "false",
            "ros2_control": "true",
            "right_can_interface": context.perform_substitution(right_can_interface),
            "left_can_interface":  context.perform_substitution(left_can_interface),
        }
    ).toprettyxml(indent="  ") # type: ignore[union-attr]


def launch_robot_nodes(context: LaunchContext,
                       right_can_interface,
                       left_can_interface,
                       controllers_file):
    robot_description = generate_robot_description(
        context, right_can_interface, left_can_interface)

    robot_description_param = {"robot_description": robot_description}
    controllers_file_str = context.perform_substitution(controllers_file)

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub, control_node]


def spawner(arguments: list) -> Node:
    """Helper to create a controller spawner node."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments + ["--controller-manager", "/controller_manager"],
    )


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can4",
            description="CAN interface for the right arm (e.g. can0).",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can5",
            description="CAN interface for the left arm (e.g. can1).",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_cartesian_controllers.yaml",
            description="Controller config file name (looked up in openarm_bringup/config/controllers/).",
        ),
    ]

    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface  = LaunchConfiguration("left_can_interface")
    controllers_file    = PathJoinSubstitution([
        FindPackageShare("openarm_cartesian_control"),
        "config",
        LaunchConfiguration("controllers_file"),
    ])

    robot_nodes = OpaqueFunction(
        function=launch_robot_nodes,
        args=[right_can_interface, left_can_interface, controllers_file],
    )

    rosbag_recorder = Node(
        package="rosbag_recorder",
        executable="recorder_node",
        name="rosbag_recorder",
        output="screen",
        parameters=[{
            "output_dir": os.path.expanduser("~/rosbags"),
            "storage_id": "mcap",
        }],
    )

    start_recording = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/rosbag_recorder/start",
                    "std_srvs/srv/Trigger", "{}",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription(declared_arguments + [
        robot_nodes,
        rosbag_recorder,
        start_recording,

        # Joint state broadcaster must come up before controllers read joint state
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
            spawner(["joint_state_broadcaster"]),
        ]),

        # Cartesian impedance controllers (one per arm)
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
            spawner([
                "left_cartesian_impedance_controller",
                "right_cartesian_impedance_controller",
            ]),
        ]),

        # Gripper controllers
        TimerAction(period=LAUNCH_DELAY_SECONDS, actions=[
            spawner(["left_gripper_controller", "right_gripper_controller"]),
        ]),
    ])