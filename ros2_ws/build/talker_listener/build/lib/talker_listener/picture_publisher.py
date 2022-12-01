import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
import threading

class CameraBufferFlusherThread(threading.Thread):
    def __init__(self, camera, name='OcvBufferFlusher'):
        self.camera = camera
        self.last_frame = None
        super(CameraBufferFlusherThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()

class MinimalPublisher(Node):
    qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=2)
    iCounter = 0
    cap = cv.VideoCapture(0)
    cam_cleaner = CameraBufferFlusherThread(cap)
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'imagePi', self.qosProfile)
        timer_period = .2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.im_list = []       
        self.bridge = CvBridge()

        
        while self.cam_cleaner.last_frame is None:
                    Kartoffel = 1 # Mach nichts bis ein frame da ist
        

    def timer_callback(self):
        
        cv_image = self.cam_cleaner.last_frame
        cv.resize(cv_image,(int(cv_image.shape[1]/4),int(cv_image.shape[0]/4)),interpolation=cv.INTER_LINEAR)
        #print(success)
        cv.putText(cv_image, f'{self.iCounter}',(50,50), cv.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), thickness=2)
        print(self.iCounter)
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        cv.imwrite("img_last_send.jpg",cv_image)
        self.get_logger().info('Publishing an image')
        self.iCounter += 1
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    #minimal_publisher.cap.release()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()