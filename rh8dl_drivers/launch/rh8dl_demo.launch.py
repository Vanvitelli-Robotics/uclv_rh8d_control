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
            default_value="false",
            description="start with simulated hand",
        )
    )

    fake_hand = LaunchConfiguration("fake_hand")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rh8d_description"), "urdf", "rh8dl_only_hc.urdf.xacro"]
            ),
            " ",
            "Fake_hardware:=",
            fake_hand,
            " ",
        ]
    )
    robot_description = {
            "robot_description": ParameterValue(value=robot_description_content, value_type=str)
        }
    rh8dl_controllers = PathJoinSubstitution(
        [FindPackageShare("rh8dl_drivers"), "config", "rh8dl_demo_controllers.yaml"]
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
    fingertip_sensors_broadcaster_spawner = Node(
       package="controller_manager",
       executable="spawner",
       arguments=["fts_broadcaster", "--controller-manager", "/rh8dl/controller_manager"],
    )
    palm_sensor_broadcaster_spawner = Node(
       package="controller_manager",
       executable="spawner",
       arguments=["tof_broadcaster", "--controller-manager", "/rh8dl/controller_manager"],
    )

    grasping_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["grasping_controller", "--controller-manager", "/rh8dl/controller_manager" , "--inactive"],
                output="screen",
    )
            
    integrator_names = ["thumb_integrator", "index_integrator", "middle_integrator", "ring_integrator"]
    
    # Lista di nodi spawner per ogni dito
    integrator_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/rh8dl/controller_manager"]
        )
        for name in integrator_names
    ]

    proportional_names = ["thumb_force_sensor_proportional", "index_force_sensor_proportional", 
                          "middle_force_sensor_proportional", "ring_force_sensor_proportional"]

    # List of spawners for proportional control
    proportional_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/rh8dl/controller_manager" , "--inactive"]
        )
        for name in proportional_names
    ]

    slipping_avoidance_names = [ "thumb_force_sensor_sa",  "index_force_sensor_sa" , 
                                "middle_force_sensor_sa", "ring_force_sensor_sa" ]

    # List of spawners for slipping avoidance control
    slipping_aavoidance_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/rh8dl/controller_manager" , "--inactive"]
        )
        for name in slipping_avoidance_names
    ]
    nodes = [
        control_node,
        hand_joint_state_broadcaster_spawner,
        hand_joint_trajectory_controller_spawner,
        fingertip_sensors_broadcaster_spawner,
        palm_sensor_broadcaster_spawner,
        grasping_controller_spawner
    ] + integrator_spawners + proportional_spawners + slipping_aavoidance_spawners

    return LaunchDescription(declared_arguments + nodes)
