# Copyleft 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "active_palm_sensor",
            default_value="true",
            description="Active the palm sensor of the hand",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    active_palm_sensor = LaunchConfiguration("active_palm_sensor")


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_left_hand"), "urdf", "hand.urdf.xacro"]),     
                " ",
                "active_palm_sensor:=",
                active_palm_sensor,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_left_hand"),
            "config",
            "base_hand_controller.yaml",
        ]
    )


#    rviz_config_file = PathJoinSubstitution(
#        [FindPackageShare("ros2_control_left_hand"), "diffbot/rviz", "diffbot.rviz"]
#    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
#        arguments=["--ros-args", "--log-level", "debug"],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_content, 
                    }]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
      #  arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    hand_joint_state_broadcaster_spawner = Node(
       package="controller_manager",
       executable="spawner",
       arguments=["hand_joint_state_broadcaster", "--controller-manager", "/controller_manager"],

    )

    grasping_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["grasping_controller", "--controller-manager", "/controller_manager"],
                name="grasping_controller_spawner",
                output="screen",
    )
        
    fingertip_sensors_broadcaster_spawner = Node(
       package="controller_manager",
       executable="spawner",
       arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],

    )

    palm_sensor_broadcaster_spawner = Node(
       package="controller_manager",
       executable="spawner",
       arguments=["tof_broadcaster", "--controller-manager", "/controller_manager"],

    )
    
    integrator_names = ["thumb_integrator", "index_integrator", "middle_integrator", "ring_integrator"]
    
    # Lista di nodi spawner per ogni dito
    integrator_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"]
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
            arguments=[name, "--controller-manager", "/controller_manager" , "--inactive"]
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
            arguments=[name, "--controller-manager", "/controller_manager" , "--inactive"]
        )
        for name in slipping_avoidance_names
    ]


#    controller_spawners_handler = RegisterEventHandler(
#        event_handler=OnExecutionComplete(
#            target_action=control_node,  # Quando controller_manager inizia
#            on_completion=[
#                TimerAction(
#                    period= 5.0,
#                    actions = [ grasping_controller_spawner,  hand_joint_state_broadcaster_spawner, fingertip_sensors_broadcaster_spawner, 
#                        palm_sensor_broadcaster_spawner, integrator_spawner, proportional_spawner],
#                )]  # Avvia entrambi i controller in parallelo
#        )
#    )

    nodes = [
        control_node,
        robot_state_pub_node,
        hand_joint_state_broadcaster_spawner,
        fingertip_sensors_broadcaster_spawner,
        grasping_controller_spawner,
        palm_sensor_broadcaster_spawner,
        #controller_spawners_handler
    ] + integrator_spawners + proportional_spawners + slipping_aavoidance_spawners

    return LaunchDescription(declared_arguments + nodes)
