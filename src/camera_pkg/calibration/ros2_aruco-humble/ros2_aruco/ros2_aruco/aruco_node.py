"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

    /tf (tf2_msgs.msg.TFMessage) # <--- 新增：现在会发布TF变换
       Transforms of detected markers

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)
    publish_tf - whether to publish marker poses as TF transforms (default False) # <--- 新增参数说明

Author: Nathan Sprague
Version: 10/26/2020 (Modified by Gemini for TF broadcast and OpenCV 4.x compatibility)

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
import tf2_ros # <--- 新增导入

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import TransformStamped # <--- 新增导入


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        # publish_tf 参数声明
        self.declare_parameter(
            name="publish_tf",
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether to publish marker poses as TF transforms.",
            ),
        )
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.get_logger().info(f"Param: publish_tf = {self.publish_tf}")

        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )
        # 新增：image_is_rectified 参数声明
        self.declare_parameter(
            name="image_is_rectified",
            value=True, # 默认值 True
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether the input image is rectified.",
            ),
        )
        self.image_is_rectified = self.get_parameter("image_is_rectified").get_parameter_value().bool_value
        self.get_logger().info(f"Param: image_is_rectified = {self.image_is_rectified}")


        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Make sure we have a valid dictionary id:
        try:
            # 更改为 getPredefinedDictionary 兼容 OpenCV 4.x
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            # 兼容旧版本 aruco
            if not isinstance(self.aruco_dictionary, cv2.aruco.Dictionary):
                 raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))
            # 退出节点或采取其他错误处理，因为字典无效无法继续
            rclpy.shutdown()
            exit(1)


        self.aruco_parameters = cv2.aruco.DetectorParameters() # 兼容 OpenCV 4.x
        
        self.bridge = CvBridge()

        # TF Broadcaster 初始化 <--- 新增
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        # 确保输入图像的编码与 OpenCV 期望的匹配，或进行适当转换
        # mono8 是黑白图像，如果输入是彩色图像，可能需要 bgr8 或 rgb8
        # ros2_aruco 原始版本可能期望 mono8
        # 如果 L515 发布的是 bgr8，这里可能需要调整
        # cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")

        # 假设输入是彩色图像（如bgr8），通常aruco在灰度图上检测
        cv_image_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        cv_image_gray = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2GRAY)


        markers = ArucoMarkers()
        pose_array = PoseArray()
        
        # 确定 TF 的父帧ID
        parent_frame_id = self.camera_frame if self.camera_frame != "" else self.info_msg.header.frame_id
        markers.header.frame_id = parent_frame_id
        pose_array.header.frame_id = parent_frame_id

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        # Use the correct image (grayscale for detectMarkers)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image_gray, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if marker_ids is not None:
            # 估计姿态
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.intrinsic_mat, self.distortion
            )
            
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

                # 发布 TF 变换 <--- 新增
                if self.publish_tf:
                    t = TransformStamped()
                    t.header.stamp = img_msg.header.stamp
                    t.header.frame_id = parent_frame_id # 父帧
                    t.child_frame_id = f"aruco_marker_{marker_id[0]}" # 子帧，使用实际ID
                    t.transform.translation.x = pose.position.x
                    t.transform.translation.y = pose.position.y
                    t.transform.translation.z = pose.position.z
                    t.transform.rotation.x = pose.orientation.x
                    t.transform.rotation.y = pose.orientation.y
                    t.transform.rotation.z = pose.orientation.z
                    t.transform.rotation.w = pose.orientation.w
                    
                    self.tf_broadcaster.sendTransform(t)

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
