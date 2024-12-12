import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
from cat.node.detect_object import get_object
from cat.node.camera_subscriber import CameraSubscriber

class Go(Node):
    
    def __init__(self):
        super().__init__("go")
        self.get_logger().info('********init************')
        # self.camera_sub = camera_subscriber # use instance of class CameraSubscriber
        # print(self.camera_sub.current_frame)
        qos_profile = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.object_info = (0,0)
        self.camera_subscriber = self.create_subscription(
            String,
            'gesture',
            self.move,
            qos_profile)
    
        self.move_subscriber = self.create_subscription(
                String,
                'move',
                self.object_move,
                qos_profile)


    def object_move(self,data):
        print(data)
        twist = Twist()
        str_data = data.data
        name = str_data.split()[0]
        if name == "Can":
            twist.angular.z=0.2
            twist.linear.x=0.2
            self.pub.publish(twist)
            time.sleep(2)
            self.stop()
        elif name == "Bottle":
            twist.angular.z=0.0
            twist.linear.x=0.2
            self.pub.publish(twist)
            time.sleep(2)
            self.stop()
        elif name == "Toothpaste":
            twist.angular.z=-0.2
            twist.linear.x=0.2
            self.pub.publish(twist)
            time.sleep(2)
            self.stop()
        self.get_logger().info(f'object_move : {name}')

    def move(self,data):
        self.get_logger().info('********Receive Gesture************')
        twist = Twist()
            
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        
        self.get_logger().info('gesture:{0}'.format(data.data))
        if data.data == "Paper": # Back
            twist.linear.x = -0.2
            twist.angular.z = 0.0
            self.pub.publish(twist)
            time.sleep(2)
            self.stop()

        elif data.data == "Rock":
            twist.linear.x=0.4
            twist.angular.z=0.0
            self.pub.publish(twist)
            time.sleep(2)
            self.stop()
            # current_frame = cam.current_frame



    def stop(self):
        twist = Twist() 
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)

    def calculate_distance(self, pixel_height):
        """
        카메라와 사물 간의 거리 계산
        :param pixel_height: 이미지에서 추출한 픽셀 높이
        :return: 사물과 카메라 간 거리 mm 단위
        600x480 사이즈에서 focal distance는 495.65로 변경 필요
        """
        distance = (165 * 2705) / pixel_height
        return distance

    ## skeleton 사물 선택이 먼저 되어야함
    def calculate_theta(self, distance, object_type):
        """
        거리를 기반으로 Theta 계산
        :param distance: 카메라와 사물 간 거리
        :param object_type: 사물을 나타내는 숫자 (1 또는 3) 2는 직진
        :return: 계산된 Theta 값 (라디안)
        """
        if object_type == 1:
            theta = -1 * math.atan(500 / distance)
        elif object_type == 3:
            theta = math.atan(500 / distance)
        else:
            raise ValueError("Invalid object type. Use 1 or 3.")
        return theta

    def calculate_control_variable(self,theta):
        """
        Theta를 기반으로 TurtleBot 제어 변수 계산
        :param theta: 도 단위의 Theta
        :return: 제어 변수 (라디안 단위)
        """
        result = (math.pi / 180) * theta
        return result




def main(args=None):
    rclpy.init(args=args)
    print("main")
    cam = CameraSubscriber()
    goto_goal = Go()
    try:  
        rclpy.spin(goto_goal)
    except KeyboardInterrupt: 
        goto_goal.get_logger().info('Error')
    finally:
        goto_goal.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
