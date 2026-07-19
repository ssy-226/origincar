from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('require_start_signal', default_value='true'),
        DeclareLaunchArgument('start_topic', default_value='/start'),

        DeclareLaunchArgument('v_line', default_value='1.0'),
        DeclareLaunchArgument('v_line_min', default_value='0.45'),
        DeclareLaunchArgument('kp_line', default_value='0.006'),
        DeclareLaunchArgument('y_line', default_value='240'),
        DeclareLaunchArgument('line_confidence', default_value='0.8'),
        DeclareLaunchArgument('line_filter_alpha', default_value='0.7'),
        DeclareLaunchArgument('line_deadband', default_value='3.0'),
        DeclareLaunchArgument('line_max_angular', default_value='5.0'),
        DeclareLaunchArgument('line_angular_slew_rate', default_value='12.0'),
        DeclareLaunchArgument('line_turn_slowdown', default_value='0.35'),
        DeclareLaunchArgument('return_delay', default_value='8.0'),

        DeclareLaunchArgument('v_avoid', default_value='0.8'),
        DeclareLaunchArgument('v_avoid_min', default_value='0.45'),
        DeclareLaunchArgument('kp_avoid', default_value='0.0035'),
        DeclareLaunchArgument('detection_confidence', default_value='0.8'),
        DeclareLaunchArgument('avoid_filter_alpha', default_value='0.7'),
        DeclareLaunchArgument('avoid_direction_lock', default_value='0.35'),
        DeclareLaunchArgument('avoid_release_timeout', default_value='0.25'),
        DeclareLaunchArgument('avoid_max_angular', default_value='5.0'),
        DeclareLaunchArgument('avoid_angular_slew_rate', default_value='16.0'),
        DeclareLaunchArgument('avoid_turn_slowdown', default_value='0.25'),
        DeclareLaunchArgument('p_confirm_frames', default_value='3'),
        DeclareLaunchArgument('p_stop_after_qrcode', default_value='true'),

        DeclareLaunchArgument('y_zt', default_value='155'),
        DeclareLaunchArgument('y_p', default_value='435'), 
        DeclareLaunchArgument('y_qrcode', default_value='167'),

        DeclareLaunchArgument(
            'aurora_detection_topic',
            default_value='/aurora/person_detection',
        ),
        DeclareLaunchArgument('v_qrcode', default_value='0.45'),
        DeclareLaunchArgument('v_qrcode_min', default_value='0.25'),
        DeclareLaunchArgument('kp_qrcode', default_value='0.005'),
        DeclareLaunchArgument('qrcode_confidence', default_value='0.7'),
        DeclareLaunchArgument('qrcode_min_bottom_y', default_value='40'),
        DeclareLaunchArgument('qrcode_filter_alpha', default_value='0.65'),
        DeclareLaunchArgument('qrcode_deadband', default_value='4.0'),
        DeclareLaunchArgument('qrcode_max_angular', default_value='2.5'),
        DeclareLaunchArgument('qrcode_angular_slew_rate', default_value='10.0'),
        DeclareLaunchArgument('qrcode_turn_slowdown', default_value='0.35'),

        DeclareLaunchArgument('control_period', default_value='0.03'),
        DeclareLaunchArgument('resnet_timeout', default_value='0.25'),
        DeclareLaunchArgument('yolo_timeout', default_value='0.18'),
        DeclareLaunchArgument('qrcode_approach_timeout', default_value='0.20'),
        DeclareLaunchArgument('qrcode_cooldown', default_value='5.0'),
        DeclareLaunchArgument('qr_stop_duration', default_value='0.09'),
        DeclareLaunchArgument('qr_maneuver_duration', default_value='1.11'),
        DeclareLaunchArgument('qr_reverse_speed', default_value='-0.8'),
        DeclareLaunchArgument('qr_angular_speed', default_value='5.0'),
        DeclareLaunchArgument('person_event_topic', default_value='/person_detected'),
        DeclareLaunchArgument('person_slowdown_factor', default_value='0.45'),
        DeclareLaunchArgument('person_ramp_down', default_value='0.35'),
        DeclareLaunchArgument('person_hold_duration', default_value='0.80'),
        DeclareLaunchArgument('person_ramp_up', default_value='0.80'),
        DeclareLaunchArgument('person_slowdown_cooldown', default_value='2.0'),
        
        Node(
            package='control',
            executable='avoid',
            name='avoid',
            output='screen',
            parameters=[{
                'require_start_signal': ParameterValue(
                    LaunchConfiguration('require_start_signal'),
                    value_type=bool,
                ),
                'start_topic': LaunchConfiguration('start_topic'),
                'v_avoid': LaunchConfiguration('v_avoid'),
                'v_avoid_min': LaunchConfiguration('v_avoid_min'),
                'kp_avoid': LaunchConfiguration('kp_avoid'),
                'detection_confidence': LaunchConfiguration('detection_confidence'),
                'avoid_filter_alpha': LaunchConfiguration('avoid_filter_alpha'),
                'avoid_direction_lock': LaunchConfiguration('avoid_direction_lock'),
                'avoid_release_timeout': LaunchConfiguration('avoid_release_timeout'),
                'max_angular': LaunchConfiguration('avoid_max_angular'),
                'angular_slew_rate': LaunchConfiguration('avoid_angular_slew_rate'),
                'turn_slowdown': LaunchConfiguration('avoid_turn_slowdown'),
                'p_confirm_frames': LaunchConfiguration('p_confirm_frames'),
                'p_stop_after_qrcode': ParameterValue(
                    LaunchConfiguration('p_stop_after_qrcode'),
                    value_type=bool,
                ),

                'y_p': LaunchConfiguration('y_p'),
                'y_zt': LaunchConfiguration('y_zt'),
                'y_qrcode': LaunchConfiguration('y_qrcode'),
            }]
        ),
        
        Node(
            package='control',
            executable='line',
            name='line',
            output='screen',
            parameters=[{
                'require_start_signal': ParameterValue(
                    LaunchConfiguration('require_start_signal'),
                    value_type=bool,
                ),
                'start_topic': LaunchConfiguration('start_topic'),
                'kp_line': LaunchConfiguration('kp_line'),
                'v_line': LaunchConfiguration('v_line'),
                'v_line_min': LaunchConfiguration('v_line_min'),
                'y_line': LaunchConfiguration('y_line'),
                'line_confidence': LaunchConfiguration('line_confidence'),
                'error_filter_alpha': LaunchConfiguration('line_filter_alpha'),
                'error_deadband': LaunchConfiguration('line_deadband'),
                'max_angular': LaunchConfiguration('line_max_angular'),
                'angular_slew_rate': LaunchConfiguration('line_angular_slew_rate'),
                'turn_slowdown': LaunchConfiguration('line_turn_slowdown'),
                'qrcode_cooldown': LaunchConfiguration('qrcode_cooldown'),
                'return_delay': LaunchConfiguration('return_delay'),
            }]
        ),

        Node(
            package='control',
            executable='qr_follow',
            name='qr_follow',
            output='screen',
            parameters=[{
                'require_start_signal': ParameterValue(
                    LaunchConfiguration('require_start_signal'),
                    value_type=bool,
                ),
                'start_topic': LaunchConfiguration('start_topic'),
                'detection_topic': LaunchConfiguration(
                    'aurora_detection_topic'
                ),
                'v_qrcode': LaunchConfiguration('v_qrcode'),
                'v_qrcode_min': LaunchConfiguration('v_qrcode_min'),
                'kp_qrcode': LaunchConfiguration('kp_qrcode'),
                'qrcode_confidence': LaunchConfiguration(
                    'qrcode_confidence'
                ),
                'qrcode_min_bottom_y': LaunchConfiguration(
                    'qrcode_min_bottom_y'
                ),
                'error_filter_alpha': LaunchConfiguration(
                    'qrcode_filter_alpha'
                ),
                'error_deadband': LaunchConfiguration('qrcode_deadband'),
                'max_angular': LaunchConfiguration('qrcode_max_angular'),
                'angular_slew_rate': LaunchConfiguration(
                    'qrcode_angular_slew_rate'
                ),
                'turn_slowdown': LaunchConfiguration(
                    'qrcode_turn_slowdown'
                ),
            }],
        ),

        Node(
            package='control',
            executable='master',
            name='master',
            output='screen',
            parameters=[{
                'require_start_signal': ParameterValue(
                    LaunchConfiguration('require_start_signal'),
                    value_type=bool,
                ),
                'start_topic': LaunchConfiguration('start_topic'),
                'control_period': LaunchConfiguration('control_period'),
                'resnet_timeout': LaunchConfiguration('resnet_timeout'),
                'yolo_timeout': LaunchConfiguration('yolo_timeout'),
                'qrcode_approach_timeout': LaunchConfiguration(
                    'qrcode_approach_timeout'
                ),
                'qrcode_cooldown': LaunchConfiguration('qrcode_cooldown'),
                'qr_stop_duration': LaunchConfiguration('qr_stop_duration'),
                'qr_maneuver_duration': LaunchConfiguration('qr_maneuver_duration'),
                'qr_reverse_speed': LaunchConfiguration('qr_reverse_speed'),
                'qr_angular_speed': LaunchConfiguration('qr_angular_speed'),
                'person_event_topic': LaunchConfiguration(
                    'person_event_topic'
                ),
                'person_slowdown_factor': LaunchConfiguration(
                    'person_slowdown_factor'
                ),
                'person_ramp_down': LaunchConfiguration('person_ramp_down'),
                'person_hold_duration': LaunchConfiguration(
                    'person_hold_duration'
                ),
                'person_ramp_up': LaunchConfiguration('person_ramp_up'),
                'person_slowdown_cooldown': LaunchConfiguration(
                    'person_slowdown_cooldown'
                ),
            }],
        ),
        
        Node(
            package='control',
            executable='qr_log',
            name='qr_log',
            output='screen',
        ),
    ])
