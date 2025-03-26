from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_hand",
            default_value="true",
            description="start with simulated hand",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="launch rviz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="rh8dl_base_controllers.yaml",
            description="controllers_file",
        )
    )

    fake_hand = LaunchConfiguration("fake_hand")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controllers_file = LaunchConfiguration("controllers_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rh8d_description"), "urdf", "rh8dl_only_hc.urdf.xacro"]
            ),
            " ",
            "FakeHardware:=",
            fake_hand,
            " ",
        ]
    )
    robot_description = {
            "robot_description": ParameterValue(value=robot_description_content, value_type=str)
        }
    rh8dl_controllers = PathJoinSubstitution(
        [FindPackageShare("rh8dl_drivers"), "config", controllers_file]
    )   
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            rh8dl_controllers,
        ],
        output="screen",
        namespace="rh8dl",
    )

    hand_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_joint_state_broadcaster", "--controller-manager", "/rh8dl/controller_manager"],
    )
    
    hand_joint_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["hand_joint_trajectory_controller", "--controller-manager", "/rh8dl/controller_manager"],
    output="screen",
    namespace="",
    )
    
    relay = Node(
            package="topic_tools",
            executable="relay",
            arguments=["/rh8dl/joint_states", "/joint_states"],
            output="screen",
        )

    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
      #  arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    
    nodes = [
        control_node,
        hand_joint_state_broadcaster_spawner,
        rviz_node,
        relay,
        hand_joint_trajectory_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
