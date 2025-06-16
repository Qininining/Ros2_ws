from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals


arg_calibration_type = DeclareLaunchArgument('calibration_type', choices=["eye_in_hand", "eye_on_base"],
                                             description="Type of the calibration")
arg_tracking_base_frame = DeclareLaunchArgument(
    'tracking_base_frame',
    default_value='camera_color_optical_frame', # <--- 添加默认值
    description='Frame of the tracking system base (e.g., camera_color_optical_frame)'
)
arg_tracking_marker_frame = DeclareLaunchArgument(
    'tracking_marker_frame',
    default_value='aruco_marker_0', # <--- 关键修改：为 marker frame 添加默认值
    description='Frame of the tracking marker (e.g., aruco_marker_0)'
)
arg_robot_base_frame = DeclareLaunchArgument(
    'robot_base_frame',
    default_value='base_link', # <--- 添加默认值
    description='Frame of the robot base (e.g., base_link)'
)
arg_robot_effector_frame = DeclareLaunchArgument(
    'robot_effector_frame',
    default_value='tool0', # <--- 添加默认值
    description='Frame of the robot effector (e.g., tool0 or ee_link)'
)

is_eye_in_hand = LaunchConfigurationEquals('calibration_type', 'eye_in_hand')
