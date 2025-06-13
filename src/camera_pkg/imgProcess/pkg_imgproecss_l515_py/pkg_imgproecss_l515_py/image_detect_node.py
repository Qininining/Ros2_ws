import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import message_filters # 用于同步图像消息

# 导入自定义消息和几何消息
from hand_tracking_msgs.msg import HandLandmarksDepth
from geometry_msgs.msg import Point

class HandDepthDetector(Node):
    """
    一个 ROS 2 节点，订阅 RGB 和对齐后的深度图像，
    使用 MediaPipe 进行手部检测，并获取检测到手部所有关键点的深度信息。
    增加安全检测：当深度值为0、21个点检测不全或超过1只手时，不发布消息。
    """
    def __init__(self):
        super().__init__('hand_depth_detector_node')
        self.get_logger().info('手部深度检测节点已启动。')

        # 初始化 CvBridge 用于 ROS Image 和 OpenCV 图像之间的转换
        self.bridge = CvBridge()

        # 初始化 MediaPipe Hands 模型
        self.mp_hands = mp.solutions.hands
        # min_detection_confidence 和 min_tracking_confidence 可以根据需要调整
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2, # 检测最多两只手
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils # 用于绘制手部关键点

        # 创建消息过滤器订阅者
        self.rgb_sub = message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        self.depth_sub = message_filters.Subscriber(self, Image, "/camera/aligned_depth_to_color/image_raw")

        # 使用 ApproximateTimeSynchronizer 进行同步
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1 # queue_size=10, slop=0.1s
        )
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info('已订阅 RGB 和对齐后的深度话题并进行同步。')

        # 创建用于显示结果的 OpenCV 窗口
        cv2.namedWindow("手部检测与深度", cv2.WINDOW_AUTOSIZE)

        # 标志，用于确保只打印一次图像尺寸信息
        self._log_dim_once = False

        # 创建一个发布者，用于发布手部关键点和深度信息
        self.hand_landmarks_pub = self.create_publisher(
            HandLandmarksDepth, # 消息类型
            'hand_landmarks_depth_data', # 发布的话题名称
            10 # QoS 队列大小
        )
        self.get_logger().info('已创建手部关键点和深度信息发布者。')


    def __del__(self):
        # 析构函数，确保关闭 OpenCV 窗口
        self.hands.close()
        cv2.destroyAllWindows()
        self.get_logger().info('手部深度检测节点已关闭。')

    def image_callback(self, rgb_msg, depth_msg):
        """
        同步图像回调函数，处理 RGB 和深度图像。
        增加安全检测，不符合条件时不发布消息。
        """
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        except Exception as e:
            self.get_logger().error(f"CvBridge 转换图像时出错: {e}")
            return
        
        if not self._log_dim_once:
            self.get_logger().info(f"RGB 图像尺寸: {rgb_image.shape[1]}x{rgb_image.shape[0]}")
            self.get_logger().info(f"对齐后的深度图像尺寸: {depth_image.shape[1]}x{depth_image.shape[0]}")
            self._log_dim_once = True

        rgb_image_mp = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image_mp)
        display_image = rgb_image.copy() # 在副本上绘制，不修改原始图像

        # 初始化 HandLandmarksDepth 消息
        hand_landmarks_msg = HandLandmarksDepth()
        hand_landmarks_msg.header = rgb_msg.header # 复制时间戳和帧ID

        # 用于存储当前帧有效手部关键点的临时列表
        valid_landmarks_for_publish = []
        should_publish_this_frame = False

        if results.multi_hand_landmarks:
            num_detected_hands = len(results.multi_hand_landmarks)

            # --- 安全检测 1: 检查是否只检测到了一只手 ---
            if num_detected_hands > 1:
                self.get_logger().warn(f"检测到 {num_detected_hands} 只手。当前帧不发布手部姿态消息 (只支持单手)。")
                # 不设置 should_publish_this_frame 为 True
            elif num_detected_hands == 1:
                hand_landmarks = results.multi_hand_landmarks[0] # 只处理第一只手

                self.get_logger().info("--- 检测到一只手 ---")
                # 绘制手部关键点和连接线
                self.mp_drawing.draw_landmarks(
                    display_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                temp_landmarks_list = []
                has_zero_depth = False

                # 遍历手部所有 21 个关键点
                for i, landmark in enumerate(hand_landmarks.landmark):
                    px = int(landmark.x * rgb_image.shape[1])
                    py = int(landmark.y * rgb_image.shape[0])

                    depth_value_mm = 0 # 深度值，毫米
                    
                    # 确保坐标在深度图像范围内
                    if 0 <= px < depth_image.shape[1] and 0 <= py < depth_image.shape[0]:
                        depth_value_mm = depth_image[py, px]
                    
                    # --- 安全检测 2: 检查深度值是否为零或负 ---
                    if depth_value_mm <= 0:
                        has_zero_depth = True
                        self.get_logger().warn(
                            f"  关键点 {self.mp_hands.HandLandmark(i).name} (x={px}, y={py}): 深度值为 {depth_value_mm}mm (无效数据)。"
                        )
                        # 如果有任何一个关键点深度无效，则此帧不发布数据
                        # 但是我们仍然会将该点添加到temp_landmarks_list中，以便后续检查21个点是否齐全
                        # 确保即使深度无效，Point的z值也为0.0，不留垃圾数据
                        point_msg = Point()
                        point_msg.x = float(px)
                        point_msg.y = float(py)
                        point_msg.z = 0.0 # 存储深度（米），无效深度为0
                        temp_landmarks_list.append(point_msg)
                        continue # 继续处理下一个关键点，但不标注在图像上
                    else:
                        # 将深度从毫米转换为米
                        depth_value_meters = float(depth_value_mm) / 1000.0 # 转换为浮点型并除以1000

                        point_msg = Point()
                        point_msg.x = float(px)
                        point_msg.y = float(py)
                        point_msg.z = depth_value_meters # 存储深度（米）
                        temp_landmarks_list.append(point_msg)

                        self.get_logger().info(
                            f"  关键点 {self.mp_hands.HandLandmark(i).name} (x={px}, y={py}): 深度 {depth_value_mm} 毫米"
                        )
                        # 在图像上标注关键点深度信息 (只选择几个关键点显示，避免图像过于拥挤)
                        if i == self.mp_hands.HandLandmark.WRIST.value: # 手腕
                            cv2.putText(display_image, f"Wrist: {depth_value_mm}mm", (px + 10, py - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                        elif i == self.mp_hands.HandLandmark.INDEX_FINGER_TIP.value: # 食指指尖
                            cv2.putText(display_image, f"Index: {depth_value_mm}mm", (px + 10, py - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                        elif i == self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP.value: # 中指指尖
                            cv2.putText(display_image, f"Middle: {depth_value_mm}mm", (px + 10, py - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
                
                # --- 安全检测 3: 检查是否检测到全部 21 个关键点 ---
                if len(temp_landmarks_list) != 21:
                    self.get_logger().warn(f"检测到的手部关键点数量不足21个 ({len(temp_landmarks_list)}/21)。当前帧不发布手部姿态消息。")
                elif has_zero_depth:
                    self.get_logger().warn("检测到手部关键点包含无效深度 (0mm)。当前帧不发布手部姿态消息。")
                else:
                    # 所有安全检测通过，可以发布此手的数据
                    valid_landmarks_for_publish = temp_landmarks_list
                    should_publish_this_frame = True # 标记此帧应该发布数据
            else: # num_detected_hands == 0
                self.get_logger().debug("当前帧未检测到手部。")
        else: # results.multi_hand_landmarks is None (no hands detected)
            self.get_logger().debug("当前帧未检测到手部。")

        # 根据 should_publish_this_frame 决定是否发布消息
        if should_publish_this_frame:
            hand_landmarks_msg.landmarks = valid_landmarks_for_publish
            self.hand_landmarks_pub.publish(hand_landmarks_msg)
            self.get_logger().info("手部关键点和深度信息已成功发布。")
        else:
            self.get_logger().debug("当前帧未满足发布条件，未发布手部姿态消息。") # Debug 级别，避免频繁输出

        # 显示结果图像
        cv2.imshow("手部检测与深度", display_image)
        cv2.waitKey(1) # 必须调用，以刷新 OpenCV 窗口并处理事件


def main(args=None):
    rclpy.init(args=args)
    node = HandDepthDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
