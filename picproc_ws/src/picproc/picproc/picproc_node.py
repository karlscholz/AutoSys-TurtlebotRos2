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

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    msg = Twist()   
    integralRot = 0
    PGainRot = 1.95567563236331
    IGainRot = 0.4375250435
    Ts = 0.1
    PGainLin = 4
    IGainLin = 1.0
    integralLin = 0
    iReceiveCounter = 0
    qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
    yLast = 0
    yCurrent = 0
    xLast = 0
    

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_ # prevent unused variable warning
        # Create the subscriber. This subscriber will receive an Image        
        self.subscription = self.create_subscription(CompressedImage,'imagePi', self.listener_callback, self.qosProfile)
        self.subscription # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
    
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving Image')
        
        # Convert ROS Image message to OpenCV image
        currImage = self.br.compressed_imgmsg_to_cv2(data)
        
        print("start cal")
        
        # calcuate cmd_vel
        print("Verreckst du da?")
        results = self.pose.process(currImage)
        #cv.imshow('image', currImage)
        #cv.imshow('prevents crashing', currImage)
        middle = currImage.shape[1]/2
        #deadzonePer = 0.2
        print(f"middle = {middle}")
        cv.line(currImage,(int(middle),0),(int(middle),currImage.shape[0]),(255,0,0),thickness=2)
        if results.pose_landmarks:
            # Watch out: x_is is relative!
            x_is = 0.5
            self.mpDraw.draw_landmarks(currImage, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
            #deadzone = deadzonePer*currImage.shape[1]
            if results.pose_landmarks.landmark[24] and results.pose_landmarks.landmark[23]:
                x_is = (results.pose_landmarks.landmark[23].x+results.pose_landmarks.landmark[24].x)/2
            elif results.pose_landmarks.landmark[24]:
                x_is = results.pose_landmarks.landmark[24].x
            elif results.pose_landmarks.landmark[23]:
                x_is = results.pose_landmarks.landmark[23].x
            else:
                print("No hiplandmarks recognized!")
                print(f"x_is_absolute = {x_is*currImage.shape[1]}")
                print(f"x_is_rel = {x_is}")

            # Compute controller effort for rotation
            error = 0.5 - x_is
            controllerEffortRot = self.PGainRot * error + self.IGainRot * self.integralRot
            # Saturation
            if controllerEffortRot > 1.82:
                controllerEffortRot = 1.82
            elif controllerEffortRot < -1.82:
                controllerEffortRot = -1.82
            # Clamping
            if abs(controllerEffortRot) < 1.82:
                self.integralRot += error*self.Ts

            self.msg.angular.z = float(controllerEffortRot)
          

            if controllerEffortRot > 0:
                cv.putText(currImage,"Left",(200,100),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                print("Left")
            elif controllerEffortRot < 0:
                cv.putText(currImage,"Right",(200,100),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                print("Right")

            y_distance = -1 
            #both visible
            if results.pose_landmarks.landmark[23] and results.pose_landmarks.landmark[24] and results.pose_landmarks.landmark[25] and results.pose_landmarks.landmark[26]:
                y_distance = (results.pose_landmarks.landmark[25].y+results.pose_landmarks.landmark[26].y)/2 - (results.pose_landmarks.landmark[23].y+results.pose_landmarks.landmark[23].y)/2
            #left visible
            elif results.pose_landmarks.landmark[23] and results.pose_landmarks.landmark[25]:
                y_distance = results.pose_landmarks.landmark[25].y - results.pose_landmarks.landmark[23].y
            #right visible
            elif results.pose_landmarks.landmark[24] and results.pose_landmarks.landmark[26]:
                y_distance = results.pose_landmarks.landmark[26].y - results.pose_landmarks.landmark[24].y


            print("#############",y_distance)

            if y_distance > 0:
                 # Compute controller effort for linear velocity
                error = 0.23 - y_distance
                controllerEffortLin = self.PGainLin * error + self.IGainLin * self.integralLin
                # Saturation
                if controllerEffortLin > 0.26:
                    controllerEffortLin = 0.26
                elif controllerEffortLin < -0.26:
                    controllerEffortLin = -0.26
                # Clamping
                if abs(controllerEffortLin )< 0.26:
                    self.integralLin += error*self.Ts

                self.msg.linear.x = float(controllerEffortLin)

                if controllerEffortLin > 0:
                    cv.putText(currImage,"Forwards",(200,200),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                    print("Forwards")
                elif controllerEffortLin < 0:
                    cv.putText(currImage,"Backwards",(200,200),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                    print("Backwards")
            
            self.yLast = controllerEffortLin
        else:
            print("No landmarks not recognized!")

            # TP
            self.yCurrent = 0.1181*self.xLast+0.8819*self.yLast
            self.yLast = self.yCurrent
            self.xLast = 0
            self.msg.linear.x = self.yCurrent
            self.msg.linear.y = 0.0
            self.msg.linear.z = 0.0
            self.msg.angular.x = 0.0
            self.msg.angular.y = 0.0
            self.msg.angular.z = 0.0
            self.integralLin = 0
            self.integralRot = 0
                
            
        
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing ang: {self.msg.angular.z}")
        self.get_logger().info(f"Publishing lin: {self.msg.linear.x}")
        cv.imshow("Live View", currImage)
        cv.waitKey(1)
        print("cmd_vel")
    

  
def main(args=None):
    try:
        rclpy.init(args=args)
        image_subscriber = ImageSubscriber()
        # Spin the node so the callback function is called.
        rclpy.spin(image_subscriber)
        

    except KeyboardInterrupt as e:
        print("\nEnded with: KeyboardInterrupt")
    except MemoryError as e:
        print("MemoryError")
    except OverflowError as e:
        print("MemoryError")
    except BaseException as e:
        print("MemoryError")
  
    image_subscriber.msg.linear.x = 0.0
    image_subscriber.msg.linear.y = 0.0
    image_subscriber.msg.linear.z = 0.0

    image_subscriber.msg.angular.x = 0.0
    image_subscriber.msg.angular.y = 0.0
    image_subscriber.msg.angular.z = 0.0
    image_subscriber.publisher_.publish(image_subscriber.msg)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()