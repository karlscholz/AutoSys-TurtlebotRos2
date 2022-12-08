# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv# OpenCV library
import numpy as np
from geometry_msgs.msg import Twist
import mediapipe as mp
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from time import sleep

class MinimalPublisher(Node):
    msg = Twist()   
    qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_ALL)
    integral = 0
    PGain = 0
    IGain = 0
    Ts = 0.2
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)#self.qosProfile)

    def readImg(self,imgRGBin):
        self.imgRGB = imgRGBin

    def calcCmd(self):
        mpDraw = mp.solutions.drawing_utils
        mpPose = mp.solutions.pose
        pose = mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        results = pose.process(self.imgRGB)
        #cv.imshow('image', self.imgRGB)
        #cv.imshow('prevents crashing', self.imgRGB)
        middle = self.imgRGB.shape[1]/2
        deadzonePer = 0.2
        print(f"middle = {middle}")
        cv.line(self.imgRGB,(int(middle),0),(int(middle),self.imgRGB.shape[0]),(255,0,0),thickness=2)
        cv.line(self.imgRGB,(int(middle-deadzonePer*middle),0),(int(middle-deadzonePer*middle),self.imgRGB.shape[0]),(0,0,255),thickness=2)
        cv.line(self.imgRGB,(int(middle+deadzonePer*middle),0),(int(middle+deadzonePer*middle),self.imgRGB.shape[0]),(0,0,255),thickness=2)
        if results.pose_landmarks:
            # Watch out: x_is is relative!
            x_is = 0.5
            deadzone = deadzonePer*self.imgRGB.shape[1]
            if results.pose_landmarks.landmark[24] and results.pose_landmarks.landmark[23]:
                x_is = (results.pose_landmarks.landmark[23].x+results.pose_landmarks.landmark[24].x)/2
            elif results.pose_landmarks.landmark[24]:
                x_is = results.pose_landmarks.landmark[24].x
            elif results.pose_landmarks.landmark[23]:
                x_is = results.pose_landmarks.landmark[23].x
            else:
                print("No hiplandmarks recognized!")
            print(f"x_is_absolute = {x_is*self.imgRGB.shape[1]}")
            print(f"x_is_rel = {x_is}")

            # Compute controller effort
            error = 0.5 - x_is
            controllerEffort = self.PGain * error + self.IGain * self.integral
            # Saturation
            if controllerEffort > 2:
                controllerEffort = 2
            elif controllerEffort < -2:
                controllerEffort = -2
            # Clamping
            if abs(controllerEffort) < 2:
                self.integral += error*self.Ts

            self.msg.angular.z = controllerEffort

            if controllerEffort > 0:
                cv.putText(self.imgRGB,"Left",(200,100),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                print("Left")
            elif controllerEffort < 0:
                cv.putText(self.imgRGB,"Right",(200,100),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                print("Right")
        else:
            print("No landmarks not recognized!")
            self.msg.linear.x = 0.0
            self.msg.linear.y = 0.0
            self.msg.linear.z = 0.0
            self.msg.angular.x = 0.0
            self.msg.angular.y = 0.0
            self.msg.angular.z = 0.0
                
            
        
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing: {self.msg.angular.z}")
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  iReceiveCounter = 0
  qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,depth=1)
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    
    self.subscription = self.create_subscription(CompressedImage,'imagePi', self.listener_callback, self.qosProfile)
    self.subscription # prevent unused variable warning
       
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.minimal_publisher = MinimalPublisher()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving Image')
    
    # Convert ROS Image message to OpenCV image
    currImage = self.br.compressed_imgmsg_to_cv2(data)
    
    # self.iReceiveCounter += 1
    # cv.putText(currImage, f'{self.iReceiveCounter}',(200,200), cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
    #cv.destroyAllWindows()


    # if self.iReceiveCounter >= 5:
    #     cv.imshow("das zweite?", currImage)
    #     cv.imshow("das zweite prevent?", currImage)
    self.minimal_publisher.readImg(currImage)
    print("start cal")
    self.minimal_publisher.calcCmd()
    cv.imshow("das erste?", self.minimal_publisher.imgRGB)
    cv.waitKey(1)
    print("cmd_vel")
    

  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the nodes
  image_subscriber = ImageSubscriber()

    
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  print ("hier wollen wir nicht hin")
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()