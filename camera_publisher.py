import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rclpy
from rclpy.node import Node

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.get_logger().info('********init************')
        self.flag = "Enable"
        qos_profile = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.camera_publisher = self.create_publisher(Image, 'camera_data', qos_profile)
        self.flag_subscriber = self.create_subscription(
            String,
            'working_flag',
            self.switch_flag,
            qos_profile)
        self.count = 0

        self.cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.cam.set(cv2.CAP_PROP_FPS, 30)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cam.isOpened():
            self.get_logger().info("Camera open failed!")
            raise Exception("Camera open failed!")
        self.br = CvBridge()
        self.publish_timer = self.create_timer(0.2, self.publish_images)
    def publish_images(self):
        if self.flag == "Disable":
            return

        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().info('Image read failed!')
            return

        # 객체 검출
        if not self.detect_object(frame):
            self.get_logger().info('No objects detected, skipping publish')
            return

        self.get_logger().info('Publishing detected object frame')
        self.camera_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

    def detect_object(self, frame):
        # 간단한 객체 검출 로직
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
        non_zero_pixels = cv2.countNonZero(thresh)
        return non_zero_pixels > 1000  # 임계값 이상이면 객체 존재

    def switch_flag(self, data):
        self.get_logger().info('Switch message: {0}'.format(data.data))
        self.flag = data.data

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.cam.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
