import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np

class MinimalPublisher(Node):
    qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=1)
    iCounter = 0
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'imagePi', self.qosProfile)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.im_list = []       
        self.bridge = CvBridge()
        

    def timer_callback(self):
        cap = cv.VideoCapture(0)
        success, cv_image = cap.read()
        cv.resize(cv_image,(int(cv_image.shape[1]/4),int(cv_image.shape[0]/4)),interpolation=cv.INTER_LINEAR)
        print(success)
        cv.putText(cv_image, f'{self.iCounter}',(100,100), cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
        print(self.iCounter)
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        cv.imwrite("img_last_send.jpg",cv_image)
        self.get_logger().info('Publishing an image')
        self.iCounter += 1
        cap.release()

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    #minimal_publisher.cap.release()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()