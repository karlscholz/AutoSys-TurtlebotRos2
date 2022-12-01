import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
=======
>>>>>>> ce2071a913b338c3caaea9f452264b1bda47ef41
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np

class MinimalPublisher(Node):
<<<<<<< HEAD
    qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=1)
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'imagePi',self.qosProfile)
        timer_period = 1  # seconds
=======
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'Image', 10)
        timer_period = 5  # seconds
>>>>>>> ce2071a913b338c3caaea9f452264b1bda47ef41
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.im_list = []       
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture(0)

    def timer_callback(self):
        success, cv_image = self.cap.read()
<<<<<<< HEAD
        cv.resize(cv_image,(int(cv_image.shape[1]/4),int(cv_image.shape[0]/4)),interpolation=cv.INTER_LINEAR)
        print(success)
=======
        print(success)
        imgRGB = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
>>>>>>> ce2071a913b338c3caaea9f452264b1bda47ef41
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv_image), "bgr8"))
        self.get_logger().info('Publishing an image')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
<<<<<<< HEAD
    minimal_publisher.cap.release()
=======
>>>>>>> ce2071a913b338c3caaea9f452264b1bda47ef41
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()