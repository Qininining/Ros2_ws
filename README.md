# Ros2_ws

控制真实 UR5 机械臂。

## 方法1: 通过 MoveIt Servo 控制真实 UR5 机械臂

### 1. 启动 UR5
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=169.254.0.10 launch_rviz:=false kinematics_params_file:="${HOME}/my_robot_calibration.yaml"
```
**注意**: 需要确保运行的ur驱动是自己包里的而不是安装在环境中的。通过命令：
```bash
ros2 pkg prefix ur_moveit_config
```
可以查看当前使用的 `ur_moveit_config` 包的路径，确保其指向的是你自己的工作空间。（ROS默认：若包内与环境内同时存在，则优先使用包内的配置）

### 2. 启动 MoveIt 和 Servo
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_servo:=true
```

### 3. 查看 Servo Node 状态
显示 `servo_node` 的状态 (正常情况下，**0** 表示无动作或准备就绪):
```bash
ros2 topic echo /servo_node/status
```

### 4. 列出所有 Controllers
```bash
ros2 control list_controllers
```

### 5. 切换并启动 `forward_position_controller`

**重要前提**: 请先在 UR 机械臂的示教器上启动 **External Control** 模式。

为了使 `moveit_servo` 能够控制机械臂，通常需要将控制器从默认的 `scaled_joint_trajectory_controller` 切换到 `forward_position_controller`。
使用以下命令进行切换：
```bash
ros2 control switch_controllers --start forward_position_controller --stop scaled_joint_trajectory_controller
```

**控制器配置说明**:
`forward_position_controller` 是 `moveit_servo` 用来发送指令的控制器。其相关配置（例如，指令输出的话题 `command_out_topic`）可以在 `ur_moveit_config` ROS 包下的 `config/ur_servo.yaml` 文件中找到。示例如下：
```yaml
# 内容节选自 ur_moveit_config/config/ur_servo.yaml
## Topic names
cartesian_command_in_topic: ~/delta_twist_cmds  # Topic for incoming Cartesian twist commands
joint_command_in_topic: ~/delta_joint_cmds # Topic for incoming joint angle commands
joint_topic: /joint_states
status_topic: ~/status # Publish status to this topic
command_out_topic: /forward_position_controller/commands # Publish outgoing commands here
```
**注意**: 更换 `/forward_position_controller/commands` 与 `/forward_velocity_controller/commands` 的同时，需要更换：
```yaml
# 内容节选自 ur_moveit_config/config/ur_servo.yaml
# What to publish? Can save some bandwidth as most robots only require positions or velocities
publish_joint_positions: true
publish_joint_velocities: false
publish_joint_accelerations: false
```
保证其一致性。

### 6. 启动 Servo Node 服务
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```

### 7. 启动状态监控
监控关节位置、速度和末端执行器位置、速度:
```bash
ros2 run pkg_ur5_control ur5_fk_subscriber_node
```

### 8. 测试 Servo Node: 订阅末端执行器位姿
订阅末端执行器的位姿，用以测试 `servo_node` 是否按预期输出指令到 `forward_position_controller`:
```bash
ros2 topic echo /forward_position_controller/commands
```

### 9. 测试 Servo Node: 发布控制命令
发布对 `servo_node` 的控制命令，测试机械臂是否响应:

*   **相对于 `tool0` 坐标系，沿 Z 轴负方向（向下）移动 0.5 米/秒:**
    ```bash
    ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'tool0'}, twist: {linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    ```

*   **相对于 `base_link` 坐标系，沿 Z 轴负方向（向下）移动 0.5 米/秒:**
    ```bash
    ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    ```

    **注意**：`stamp: now` 表示使用当前时间戳，`frame_id: 'tool0'` 或 `'base_link'` 指定坐标系。其中 `stamp` **不可以**换为 `0` 等具体值，因为这会导致 `servo_node` 无法正确处理命令。

## 方法2: 通过自定义节点控制速度 (模拟 MoveIt Servo)
### 1. 启动 UR5
*(与方法1中的步骤1相同)*
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=169.254.0.10 launch_rviz:=false kinematics_params_file:="${HOME}/my_robot_calibration.yaml"
```
*(后续步骤待补充)*

## 当前存在的问题

1.  **`ur_velocity_controller_node`** (速度控制):
    *   ***(1)***: 缺少奇异值判断 (可能导致关节速度过大失控)。
    *   ***(2)***: 缺少碰撞检测。
    *   ***(3)***: 缺少极限位置下的速度归零机制。
    *   ***(4)***: 速度计算错误，会有偏差。
2.  **`ur5_kinematics_controller_node`** (位置控制):
    *   ***(1)***: 逆解多解问题 – 微小位移可能导致关节运动幅度过大。
        *   **初步解决方案**: 后续加入解析解求解，并选取最优解作为逆解初值，以实现稳定控制。
    *   ***(2)***: 有奇异检测，但缺乏相应处理措施。
        *   **改进方向**: 增加奇异值处理、关节限制、碰撞检测等措施。
    *   ***(3)***: 旋转是按照 `tool0` 坐标进行的，但xyz移动是按照 `base_link` 坐标进行的。
3.  **`moveit_servo`** (末端执行器控制):
    *   ***(1)***: `forward_position_controller` 控制过程中会有速度的突变，导致机械臂抖动（在速度较快时出现）。
    *   ***(2)***: `forward_velocity_controller` 控制模式下不好用，还需熟练熟练再制定操作方案。还是位置轨迹控制来间接控制速度好用。

