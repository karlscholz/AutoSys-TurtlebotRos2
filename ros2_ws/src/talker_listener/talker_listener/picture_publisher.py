import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import cv2 as cv
import threading

#asynchronously pulls all frames from buffer and only keeps latest frame to prevent lag
class CameraBufferFlusherThread(threading.Thread):
    def __init__(self, camera, name='OcvBufferFlusher'):
        self.camera = camera
        self.last_frame = None
        super(CameraBufferFlusherThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()

# picture publisher node
class PicturePublisher(Node):
    qos_Profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL) # QoSProfile specifies communication between nodes
    counter = 0
    cap = cv.VideoCapture(0)    # creates an object of the raspi cam
    cam_cleaner = CameraBufferFlusherThread(cap)    # saves the images of the raspi cam
    def __init__(self):
        super().__init__('picture_publisher')
        self.publisher = self.create_publisher(CompressedImage, 'imagePi', self.qos_Profile)  # object to publish images in topic /imagePi
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)     
        self.bridge = CvBridge()

        
        while self.cam_cleaner.last_frame is None:
            pass
        

    def timer_callback(self):
        
        cv_image = self.cam_cleaner.last_frame  # get last image
        cv.resize(cv_image,(int(cv_image.shape[1]/4),int(cv_image.shape[0]/4)),interpolation=cv.INTER_LINEAR)
        cv.putText(cv_image, f'{self.counter}',(50,50), cv.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), thickness=2)
        print(self.counter)
        self.publisher.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image))
        self.get_logger().info('Publishing an image')
        self.counter += 1
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    picture_publisher = PicturePublisher()
    rclpy.spin(picture_publisher)
    picture_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()