from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # <-- important


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    ]
    gui = LaunchConfiguration("gui")

    # Build robot_description from xacro (wrap output as a string parameter)
    xacro_cmd = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("waveshare_servos"),
            "description", "urdf", "example.urdf.xacro",
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(xacro_cmd, value_type=str)  # <-- fix
    }

    # Controllers YAML
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("waveshare_servos"),
        "bringup", "config", "example_controllers.yaml",
    ])

    # RViz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("waveshare_servos"),
        "description", "rviz", "example_ws.rviz",
    ])

    # controller_manager node (needs both description + YAML)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="log",
    )

    # Robot state publisher (publishes TF for wheel links, etc.)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff drive controller (must match your YAML controller name)
    diff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base", "--controller-manager", "/controller_manager"],
    )

    # Start RViz after JS broadcaster is up
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # After diff drive starts, spawn JS broadcaster (short delay helps)
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_controller_spawner,
            on_exit=[TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        diff_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        # If you actually want the GUI joint publisher, append it here:
        # joint_state_publisher_node,
    ]

    print(f"robot_controllers: {robot_controllers}")
    return LaunchDescription(declared_arguments + nodes)
