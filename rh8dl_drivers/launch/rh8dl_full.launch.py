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
            "connected_to",
            default_value="base_link_hand",
            description="parent link of base:1",
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
    connected_to = LaunchConfiguration("connected_to")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controllers_file = LaunchConfiguration("controllers_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rh8d_description"), "urdf", "left_hand.urdf.xacro"]
            ),
            " ",
            "FakeHardware:=",
            fake_hand,
            " ",
            "connected_to:=",
            connected_to,
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
    )
    
    
    relay = Node(
            package="topic_tools",
            executable="relay",
            arguments=["/rh8dl/joint_states", "/joint_states"],
            output="screen",
        )
    

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
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
        robot_state_pub_node,
        rviz_node,
        hand_joint_trajectory_controller_spawner,
        relay,
    ]

    return LaunchDescription(declared_arguments + nodes)
