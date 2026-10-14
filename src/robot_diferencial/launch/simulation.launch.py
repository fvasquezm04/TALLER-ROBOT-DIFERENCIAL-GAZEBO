import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    robot_package = get_package_share_directory(
        "robot_diferencial"
    )

    ros_gz_sim_package = get_package_share_directory(
        "ros_gz_sim"
    )

    xacro_file = os.path.join(
        robot_package,
        "urdf",
        "robot.urdf.xacro",
    )

    world_file = os.path.join(
        robot_package,
        "worlds",
        "pista.sdf",
    )

    rviz_config_file = os.path.join(
        robot_package,
        "rviz",
        "robot.rviz",
    )

    gazebo_launch_file = os.path.join(
        ros_gz_sim_package,
        "launch",
        "gz_sim.launch.py",
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_file
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-world",
            "pista",

            "-topic",
            "/robot_description",

            "-name",
            "robot_diferencial",

            "-x",
            "-2.60",

            "-y",
            "-2.50",

            "-z",
            "0.20",

            "-Y",
            "0.0",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[
            spawn_robot,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config_file,
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    cinematica_diferencial = Node(
        package="robot_diferencial",
        executable="cinematica_diferencial",
        name="cinematica_diferencial",
        output="screen",
        parameters=[
            {
                "radio_rueda": 0.08,
                "separacion_ruedas": 0.36,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            bridge,
            delayed_spawn,
            rviz,
            cinematica_diferencial,
        ]
    )