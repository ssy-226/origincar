from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(package) + '/launch/' + launch_file
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    device = LaunchConfiguration('device')
    aurora_rgb = LaunchConfiguration('aurora_rgb_topic')
    aurora_detect = LaunchConfiguration('aurora_detect_topic')

    args = [
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'aurora_rgb_topic',
            default_value='/rgb/image_raw',
        ),
        DeclareLaunchArgument(
            'aurora_detect_topic',
            default_value='/aurora/person_detection',
        ),
    ]

    rosbridge = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'rosbridge_server',
            'rosbridge_websocket_launch.xml',
        ],
        output='screen',
    )

    camera = include(
        'hobot_usb_cam',
        'hobot_usb_cam.launch.py',
        {
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_zero_copy': 'True',
            'usb_video_device': device,
        },
    )

    decode = include(
        'hobot_codec',
        'hobot_codec_decode.launch.py',
        {
            'codec_channel': '1',
            'codec_in_format': 'jpeg',
            'codec_out_format': 'nv12',
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/nv12_img',
        },
    )

    encode = include(
        'hobot_codec',
        'hobot_codec_encode.launch.py',
        {
            'codec_channel': '2',
            'codec_jpg_quality': '70.0',
            'codec_output_framerate': '30',
            'codec_in_format': 'nv12',
            'codec_out_format': 'jpeg',
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/nv12_img',
            'codec_pub_topic': '/jpeg_img',
        },
    )

    web = include(
        'websocket',
        'websocket.launch.py',
        {
            'websocket_image_topic': '/jpeg_img',
            'websocket_image_type': 'mjpeg',
            'websocket_smart_topic': '/detect',
        },
    )

    track = include('track', 'track.launch.py')
    detect_mono = include('detect', 'detect.launch.py')
    detect_aurora = include(
        'detect',
        'aurora.launch.py',
        {
            'rgb_topic': aurora_rgb,
            'output_topic': aurora_detect,
        },
    )
    base = include('origincar_base', 'origincar_bringup.launch.py')

    qr = Node(
        package='qr',
        executable='qr_node',
        name='qr',
        output='screen',
        parameters=[{'rgb_topic': aurora_rgb}],
    )

    vlm = Node(
        package='vlm',
        executable='vlm',
        name='vlm',
        output='screen',
        parameters=[{
            'image_topic': aurora_rgb,
            'detection_topic': aurora_detect,
        }],
    )

    return LaunchDescription(args + [
        rosbridge,
        camera,
        decode,
        encode,
        qr,
        track,
        detect_mono,
        detect_aurora,
        base,
        web,
        vlm,
    ])
