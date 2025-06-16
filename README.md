# ros2_ws

# ROS2 工作空间
# 主要用于 UR5 机械臂控制、相机标定、视觉伺服等功能

---

## 目录

- [ros2\_ws](#ros2_ws)
- [ROS2 工作空间](#ros2-工作空间)
- [主要用于 UR5 机械臂控制、相机标定、视觉伺服等功能](#主要用于-ur5-机械臂控制相机标定视觉伺服等功能)
  - [目录](#目录)
  - [快速运行指令](#快速运行指令)
    - [机械臂控制](#机械臂控制)
    - [相机相关](#相机相关)
    - [标定相关](#标定相关)
  - [UR5控制方法](#ur5控制方法)
    - [方法一：通过 MoveIt Servo 控制 UR5](#方法一通过-moveit-servo-控制-ur5)
    - [方法二：自定义节点控制速度（模拟 MoveIt Servo）](#方法二自定义节点控制速度模拟-moveit-servo)
  - [手眼标定](#手眼标定)
    - [ros2\_aruco 配置](#ros2_aruco-配置)
    - [easy\_handeye2 配置](#easy_handeye2-配置)
    - [visp配置（可选，无用）](#visp配置可选无用)
  - [当前存在的问题](#当前存在的问题)
  - [版本信息](#版本信息)

---

## 快速运行指令

### 机械臂控制

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=169.254.0.10 launch_rviz:=false kinematics_params_file:="${HOME}/my_robot_calibration.yaml"
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_servo:=true
ros2 topic echo /servo_node/status
ros2 control list_controllers
ros2 control switch_controllers --start forward_position_controller --stop scaled_joint_trajectory_controller
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
ros2 run pkg_ur5_control ur5_fk_subscriber_node
ros2 topic echo /forward_position_controller/commands
ros2 run pkg_ur_servo_cpp ur_servo_velocity_node
```

### 相机相关

```bash
ros2 launch realsense2_camera my_rs_launch.py
ros2 run pkg_imgproecss_l515_py image_detect_node
ros2 run pkg_ibvs_cpp ibvs_feature_node
ros2 topic echo /hand_landmarks_depth_data
```

### 标定相关

先开启相机和ur5机械臂，然后运行以下命令：

```bash
ros2 launch ros2_aruco my_aruco_recognition.launch.py
ros2 topic echo /aruco_poses
/opt/ros/humble/lib/rqt_tf_tree/rqt_tf_tree
ros2 launch easy_handeye2 my_calibration.launch.py calibration_type:="eye_in_hand" name:="ur5_l515_eih_calib" robot_base_frame:="base_link" robot_effector_frame:="tool0" tracking_base_frame:="camera_color_optical_frame" tracking_target_frame:="aruco_marker_0" freehand_robot_movement:=true
```

---

## UR5控制方法

### 方法一：通过 MoveIt Servo 控制 UR5

1. **启动 UR5**

    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=169.254.0.10 launch_rviz:=false kinematics_params_file:="${HOME}/my_robot_calibration.yaml"
    ```

    > **注意：**  
    > 确保使用的是自己工作空间内的 ur 驱动包。  
    > 可用 `ros2 pkg prefix ur_moveit_config` 检查路径。

2. **启动 MoveIt 和 Servo**

    ```bash
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_servo:=true
    ```

3. **查看 Servo Node 状态**

    ```bash
    ros2 topic echo /servo_node/status
    ```
    > **0** 表示无动作或准备就绪。

4. **列出所有 Controllers**

    ```bash
    ros2 control list_controllers
    ```

5. **切换并启动 forward_position_controller**

    > **前提：** UR 示教器上需启动 External Control 模式。

    ```bash
    ros2 control switch_controllers --start forward_position_controller --stop scaled_joint_trajectory_controller
    ```

    **控制器配置说明：**  
    `forward_position_controller` 是 moveit_servo 的默认输出控制器。相关配置在 `ur_moveit_config/config/ur_servo.yaml`：

    ```yaml
    # 节选
    command_out_topic: /forward_position_controller/commands
    publish_joint_positions: true
    publish_joint_velocities: false
    publish_joint_accelerations: false
    ```

6. **启动 Servo Node 服务**

    ```bash
    ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
    ```

7. **启动状态监控**

    ```bash
    ros2 run pkg_ur5_control ur5_fk_subscriber_node
    ```

8. **订阅末端执行器位姿**

    ```bash
    ros2 topic echo /forward_position_controller/commands
    ```

9. **发布控制命令测试机械臂响应**

    - **相对于 tool0 坐标系，Z 轴负方向移动 0.5 m/s：**

        ```bash
        ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'tool0'}, twist: {linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
        ```

    - **相对于 base_link 坐标系，Z 轴负方向移动 0.5 m/s：**

        ```bash
        ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
        ```

    > **注意：**  
    > `stamp: now` 必须为当前时间戳，不能为 0。

---

### 方法二：自定义节点控制速度（模拟 MoveIt Servo）

1. **启动 UR5**

    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=169.254.0.10 launch_rviz:=false kinematics_params_file:="${HOME}/my_robot_calibration.yaml"
    ```
    > *(后续步骤待补充)*

---

## 手眼标定

ros2_aruco 和 easy_handeye2 都在 github 上有 humble 版本，直接下载即可。

### ros2_aruco 配置

1. 新建 `my_aruco_recognition.launch.py` 和 `my_aruco_parameters.yaml` 文件：

    ```yaml
    # my_aruco_parameters.yaml
    /aruco_node:
      ros__parameters:
        marker_size: 0.028
        aruco_dictionary_id: DICT_4X4_250
        image_topic: /camera/color/image_raw
        camera_info_topic: /camera/color/camera_info
        publish_tf: True
        image_is_rectified: True
    ```

2. 将 aruco 的结果发送到 tf 话题。

### easy_handeye2 配置

1. 在 common_launch.py 中添加默认值：

    ```python
    'robot_base_frame': 'base_link',
    'robot_effector_frame': 'tool0',
    'tracking_base_frame': 'camera_color_optical_frame',
    'tracking_target_frame': 'aruco_marker_0',
    ```

2. 新增 `my_calibration.launch.py` 文件，并更新 setup.py 配置。

**使用操作：**  
先开启相机和ur5机械臂，然后运行以下命令：

```bash
ros2 launch ros2_aruco my_aruco_recognition.launch.py
ros2 topic echo /aruco_poses
/opt/ros/humble/lib/rqt_tf_tree/rqt_tf_tree
ros2 launch easy_handeye2 my_calibration.launch.py calibration_type:="eye_in_hand" name:="ur5_l515_eih_calib" robot_base_frame:="base_link" robot_effector_frame:="tool0" tracking_base_frame:="camera_color_optical_frame" tracking_target_frame:="aruco_marker_0" freehand_robot_movement:=true
```

### visp配置（可选，无用）

- 安装 visp 包：[github.com/lagadic/vision_visp](https://github.com/lagadic/vision_visp)
- 注释掉 `src/camera_pkg/calibration/vision_visp-rolling/visp_tracker` 中 CMakeLists.txt 的 bag 下载部分。

---

## 当前存在的问题

**`ur_velocity_controller_node`（速度控制）**

- 缺少奇异值判断（可能导致关节速度过大失控）
- 缺少碰撞检测
- 缺少极限位置下的速度归零机制
- 速度计算有偏差

---

**`ur5_kinematics_controller_node`（位置控制）**

- 逆解多解问题，微小位移可能导致关节大幅运动  
  > **初步方案：** 后续加入解析解求解，选取最优解作为逆解初值
- 有奇异检测但无处理措施  
  > **改进方向：** 增加奇异值处理、关节限制、碰撞检测
- 旋转按 tool0 坐标，xyz 移动按 base_link 坐标

---

**`moveit_servo`（末端控制）**

- `forward_position_controller` 控制时有速度突变，机械臂抖动
- `forward_velocity_controller` 控制模式下体验不佳，建议用位置轨迹控制间接控制速度

---

## 版本信息

- realsense sdk：2.51.1
- realsense ros2：4.51.1

