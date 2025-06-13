import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import message_filters # 用于同步图像消息

class HandDepthDetector(Node):
    """
    一个 ROS 2 节点，订阅 RGB 和对齐后的深度图像，
    使用 MediaPipe 进行手部检测，并获取检测到手部所有关键点的深度信息。
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
        # 订阅彩色图像话题
        self.rgb_sub = message_filters.Subscriber(self, Image, "/camera/color/image_raw")
        # 订阅对齐到彩色图像的深度话题
        self.depth_sub = message_filters.Subscriber(self, Image, "/camera/aligned_depth_to_color/image_raw")

        # 使用 ApproximateTimeSynchronizer 进行同步
        # Queue size of 10 for synchronized messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1 # queue_size=10, slop=0.1s
        )
        # 注册同步回调函数
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info('已订阅 RGB 和对齐后的深度话题并进行同步。')

        # 创建用于显示结果的 OpenCV 窗口
        cv2.namedWindow("手部检测与深度", cv2.WINDOW_AUTOSIZE)

        # 标志，用于确保只打印一次图像尺寸信息
        self._log_dim_once = False

    def __del__(self):
        # 析构函数，确保关闭 OpenCV 窗口
        self.hands.close()
        cv2.destroyAllWindows()
        self.get_logger().info('手部深度检测节点已关闭。')

    def image_callback(self, rgb_msg, depth_msg):
        """
        同步图像回调函数，处理 RGB 和深度图像。
        """
        try:
            # 将 ROS RGB 图像消息转换为 OpenCV BGR 图像
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            # 将 ROS 深度图像消息转换为 OpenCV 16UC1 图像 (毫米为单位)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        except Exception as e:
            self.get_logger().error(f"CvBridge 转换图像时出错: {e}")
            return
        
        # 首次打印图像尺寸信息，验证 RGB 和深度图像是否已对齐
        if not self._log_dim_once:
            self.get_logger().info(f"RGB 图像尺寸: {rgb_image.shape[1]}x{rgb_image.shape[0]}")
            self.get_logger().info(f"对齐后的深度图像尺寸: {depth_image.shape[1]}x{depth_image.shape[0]}")
            self._log_dim_once = True

        # 将 BGR 图像转换为 RGB，MediaPipe 期望 RGB 格式
        rgb_image_mp = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        # 对图像进行手部检测
        results = self.hands.process(rgb_image_mp)

        # 绘制检测结果并获取深度
        display_image = rgb_image.copy() # 在副本上绘制，不修改原始图像

        if results.multi_hand_landmarks:
            for hand_id, hand_landmarks in enumerate(results.multi_hand_landmarks):
                self.get_logger().info(f"--- 检测到第 {hand_id + 1} 只手 ---")
                # 绘制手部关键点和连接线
                self.mp_drawing.draw_landmarks(
                    display_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # 遍历手部所有 21 个关键点
                for i, landmark in enumerate(hand_landmarks.landmark):
                    # 将归一化坐标转换为像素坐标
                    # MediaPipe 提供的坐标是 0-1 范围的比例值
                    px = int(landmark.x * rgb_image.shape[1])
                    py = int(landmark.y * rgb_image.shape[0])

                    depth_value = 0
                    # 确保坐标在深度图像范围内
                    if 0 <= px < depth_image.shape[1] and 0 <= py < depth_image.shape[0]:
                        depth_value = depth_image[py, px]
                    
                    if depth_value > 0:
                        self.get_logger().info(
                            f"  关键点 {self.mp_hands.HandLandmark(i).name} (x={px}, y={py}): 深度 {depth_value} 毫米"
                        )
                        # 在图像上标注关键点深度信息 (只选择几个关键点显示，避免图像过于拥挤)
                        if i == self.mp_hands.HandLandmark.WRIST.value: # 手腕
                            cv2.putText(display_image, f"Wrist: {depth_value}mm", (px + 10, py - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                        elif i == self.mp_hands.HandLandmark.INDEX_FINGER_TIP.value: # 食指指尖
                             cv2.putText(display_image, f"Index: {depth_value}mm", (px + 10, py - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                        elif i == self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP.value: # 中指指尖
                             cv2.putText(display_image, f"Middle: {depth_value}mm", (px + 10, py - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
                    else:
                        self.get_logger().warn(
                            f"  关键点 {self.mp_hands.HandLandmark(i).name} (x={px}, y={py}): 深度值为 0mm 或超出边界 (无效数据)。"
                        )
        else:
            self.get_logger().debug("当前帧未检测到手部。")

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
