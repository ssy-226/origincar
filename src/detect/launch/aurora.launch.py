import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_topic = LaunchConfiguration("rgb_topic")
    output_topic = LaunchConfiguration("output_topic")
    config_file = LaunchConfiguration("config_file")
    default_config = os.path.join(
        get_package_share_directory('detect'),
        'config',
        'config.json',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/rgb/image_raw",
            description="Aurora RGB sensor_msgs/Image topic",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/aurora/person_detection",
            description="Detection topic used by the person VLM node",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YOLO model configuration file",
        ),
        Node(
            package="detect",
            executable="detect_node",
            name="detect_aurora",
            output="screen",
            parameters=[{
                "is_shared_mem_sub": False,
                "sub_img_topic": rgb_topic,
                "config_file": config_file,
            }],
            remappings=[
                ("/detect", output_topic),
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
    ])
