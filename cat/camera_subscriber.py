import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cat.node.detect_gesture import Gesture
from cat.node.detect_object import get_object



class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('Camera_subscrixber')

        print("sub")
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(depth=30,  # 메시지 큐의 크기를 늘려 대기열 확보
                reliability=ReliabilityPolicy.BEST_EFFORT,  # 손실 허용 (최대한 빠른 전송)
                durability=DurabilityPolicy.VOLATILE  # 현재 활성 구독자에게만 메시지 전송
        )
        self.camera_subscriber = self.create_subscription(
            Image,
            'camera_data',
            self.listener_callback,
            qos_profile)
        self.camera_flag_publisher = self.create_publisher(String, 'working_flag', qos_profile)
        self.br = CvBridge()
        self.current_frame = "test"
        self.count = 0
        self.gesture = Gesture()
        self.gesture_publisher = self.create_publisher(String, 'gesture', qos_profile)
        self.move_publisher = self.create_publisher(String, 'move',qos_profile)
        self.flag_for_finger = False


    def listener_callback(self, data):
        msg = String()
        msg.data = "Disable"
        self.camera_flag_publisher.publish(msg)
        self.get_logger().info('Received message')
        self.count+=1
        self.current_frame = self.br.imgmsg_to_cv2(data)
        # cv2.imwrite("img"+str(self.count)+".jpg", current_frame)
        # print(object_result)
        result = self.gesture.detect_gesture(self.current_frame,self.count) 
        print('**************Finish detect****************')
        print(result)

        object_result = None

        if not result[0] == None and not self.flag_for_finger:
            print(result[0])
            if result[0] == "Rock":
                self.flag_for_finger = True
            msg.data = result[0]
            self.gesture_publisher.publish(msg) # self.current_frame added
        msg.data = "Enable"
        self.camera_flag_publisher.publish(msg)

        
        if self.flag_for_finger:
            object_result = get_object(self.current_frame)
        
        if object_result:
            if object_result[0] in ["Can","Bottle","Toothpaste"]:
                data = object_result[0] + " " +  str(object_result[1])
                print(data)
                msg.data = data
                self.move_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
