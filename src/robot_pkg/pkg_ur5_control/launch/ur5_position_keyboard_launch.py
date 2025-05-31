import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 配置区 ---
    # 根据您的系统选择终端模拟器及其命令前缀
    # 对于 gnome-terminal:
    terminal_prefix = 'gnome-terminal -- '
    # 对于 xterm:
    # terminal_prefix = 'xterm -e'
    # 确保您选择的终端模拟器已安装

    # --- 命令 1: pkg_keyboard_py keyboard_publisher_node ---
    keyboard_node = Node(
        package='pkg_keyboard_py',
        executable='keyboard_publisher_node',
        name='keyboard_publisher_node', # 节点名称
        output='screen',
        prefix=terminal_prefix # 在新终端中启动
    )

    # --- 命令 2: ur_robot_driver ur_control.launch.py ---
    # 解析 ${HOME} 路径
    kinematics_params_file_path = os.path.expanduser('~/my_robot_calibration.yaml')
    
    # ur_robot_driver 包的路径 (如果 ur_control.launch.py 在该包中)
    # ur_robot_driver_pkg_share = get_package_share_directory('ur_robot_driver')
    # ur_control_launch_file = os.path.join(ur_robot_driver_pkg_share, 'launch', 'ur_control.launch.py')

    # 使用 ExecuteProcess 在新终端中执行 ros2 launch 命令
    ur_control_launch_process = ExecuteProcess(
        cmd=[
            # 第一个元素是终端命令，后续是传递给终端的参数
            terminal_prefix.split()[0], # e.g., 'gnome-terminal'
            terminal_prefix.split()[1] if len(terminal_prefix.split()) > 1 else '', # e.g., '--' or '-e'
            'ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py',
            'ur_type:=ur5',
            'robot_ip:=169.254.0.10',
            'launch_rviz:=true', # launch参数通常作为字符串传递
            f'kinematics_params_file:={kinematics_params_file_path}'
        ],
        shell=False # 当 cmd 是一个列表时，通常设置为 False
    )

    # --- 命令 3: pkg_ur5_control ur5_kinematics_controller ---
    kinematics_controller_node = Node(
        package='pkg_ur5_control',
        executable='ur5_kinematics_controller',
        name='ur5_kinematics_controller_node', # 节点名称
        output='screen',
        prefix=terminal_prefix # 在新终端中启动
    )

    # --- 命令 4: pkg_ur5_control ur5_fk_subscriber_node ---
    fk_subscriber_node = Node(
        package='pkg_ur5_control',
        executable='ur5_fk_subscriber_node',
        name='ur5_fk_subscriber_node', # 节点名称
        output='screen',
        prefix=terminal_prefix # 在新终端中启动
    )

    return LaunchDescription([
        keyboard_node,
        ur_control_launch_process,
        kinematics_controller_node,
        fk_subscriber_node
    ])

if __name__ == '__main__':
    generate_launch_description()