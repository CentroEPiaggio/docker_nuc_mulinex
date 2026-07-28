from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    demo_share = get_package_share_directory("socialbot_demo0")
    description_share = get_package_share_directory("mulinex_description")
    gazebo_share = get_package_share_directory("omni_gazebo")

    default_world = os.path.join(demo_share, "worlds", "demo0_world.sdf")
    xacro_file = os.path.join(description_share, "urdf", "omnicar.xacro")
    controller_config = os.path.join(gazebo_share, "config", "omnicar_sim.yaml")
    rviz_config = os.path.join(description_share, "rviz", "config.rviz")

    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    verbosity = LaunchConfiguration("gz_verbosity")

    robot_description = Command(
        [
            "xacro ",
            xacro_file,
            " use_gazebo:=true",
            " yaml_file:=",
            controller_config,
            " lidar_3D:=false",
        ]
    )

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", world, "-r", "-v", verbosity],
        output="screen",
        additional_env={
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": os.environ.get("LD_LIBRARY_PATH", ""),
            "GZ_SIM_SYSTEM_PLUGIN_PATH": os.environ.get("LD_LIBRARY_PATH", ""),
        },
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "mulinex",
            "-topic",
            "robot_description",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            "0.08",
            "-Y",
            spawn_yaw,
        ],
        parameters=[{"use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/model/mulinex/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/joint_states_full@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        remappings=[
            ("/lidar", "/scan"),
            ("/model/mulinex/odometry", "/gz/odom_raw"),
            ("/rgbd_camera/image", "/camera/color/image_raw"),
            ("/rgbd_camera/camera_info", "/camera/color/camera_info"),
            ("/rgbd_camera/depth_image", "/camera/depth/image_raw"),
            ('/joint_states_full', '/joint_states'),
        ],
    )

    odom_adapter = Node(
        package="socialbot_demo0",
        executable="ground_truth_odom",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    cmd_vel_relay = Node(
        package="socialbot_demo0",
        executable="cmd_vel_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/cmd_vel",
                "output_topic": "/omni_controller/twist_cmd",
                "publish_rate": 20.0,
                "input_timeout": 0.50,
                "max_linear_x": 0.30,
                "max_linear_y": 0.0,
                "max_angular_z": 0.70,
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--param-file", controller_config],
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["omni_controller", "--param-file", controller_config],
    )

    controller_activator = Node(
        package="socialbot_demo0",
        executable="activate_omni_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-2.50"),
            DeclareLaunchArgument("spawn_y", default_value="-1.50"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("gz_verbosity", default_value="3"),
            gazebo,
            bridge,
            robot_state_publisher,
            odom_adapter,
            cmd_vel_relay,
            spawn_entity,
            rviz,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner, omni_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=omni_controller_spawner,
                    on_exit=[TimerAction(period=1.0, actions=[controller_activator])],
                )
            ),
        ]
    )
