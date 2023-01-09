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
    #
    #
    # Your Code
    msg = Twist()   
    qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
    #
    #
    #

    # Sampletime: Period of picture_Publisher_node
    Ts = 0.1
    # Rotation PID-Controller
    integralRot = 0
    PGainRot = 1.95567563236331
    IGainRot = 0.4375250435
    # Translation PID-Controller
    PGainLin = 4
    IGainLin = 1.0
    integralLin = 0

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        #
        #
        # Your Code
        super().__init__('image_subscriber') # Not Ros, super().__init__ calls __init__ of parent class (super() returns proxy object of parent)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_ # prevent unused variable warning
        # Create the subscriber. This subscriber will receive an Image        
        self.subscription = self.create_subscription(CompressedImage,'imagePi', self.listener_callback, self.qosProfile)
        self.subscription # prevent unused variable warning
        #
        #
        #
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()                                                                         
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
    # Callback is called when an image is received
    def listener_callback(self, data):
        """
        Callback function.
        """
        
        #
        #
        # Your Code
        self.get_logger().info('Receiving Image')   # Display the message on the console
        #
        #
        #
        
        # Convert ROS Image message to OpenCV image
        currImage = self.br.compressed_imgmsg_to_cv2(data)
        
        print("start cal")
        
        # calcuate cmd_vel
        results = self.pose.process(currImage)
        middle = currImage.shape[1]/2
        #deadzonePer = 0.2
        print(f"middle = {middle}")
        cv.line(currImage,(int(middle),0),(int(middle),currImage.shape[0]),(255,0,0),thickness=2)
        if results.pose_landmarks:
            self.mpDraw.draw_landmarks(currImage, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)


            # Watch out: x_is is relative!
            x_is = -1
            if results.pose_landmarks.landmark[24] and results.pose_landmarks.landmark[23]:
                x_is = (results.pose_landmarks.landmark[23].x+results.pose_landmarks.landmark[24].x)/2
            elif results.pose_landmarks.landmark[24]:
                x_is = results.pose_landmarks.landmark[24].x
            elif results.pose_landmarks.landmark[23]:
                x_is = results.pose_landmarks.landmark[23].x
            else:
                print("No hiplandmarks recognized!")

            ### PID Rotation
            if x_is > 0:
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

                if controllerEffortRot > 0:
                    cv.putText(currImage,"Left",(200,100),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                    print("Left")
                elif controllerEffortRot < 0:
                    cv.putText(currImage,"Right",(200,100),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                    print("Right")
            else:
                # if no x_is can be determined, set controllereffort to 0 and reset the integrator
                controllerEffortRot = 0
                self.integralRot = 0


          

            

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


            print(f"# y_distance = {y_distance} #")

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
                # Visualisation
                if controllerEffortLin > 0:
                    cv.putText(currImage,"Forwards",(200,200),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                    print("Forwards")
                elif controllerEffortLin < 0:
                    cv.putText(currImage,"Backwards",(200,200),cv.FONT_HERSHEY_TRIPLEX, 2.5, (0,255,0), thickness=2)
                    print("Backwards")
            else:
                # if no y_distance can be determined, set controllereffort to 0 and reset the integrator
                controllerEffortLin = 0
                self.integralLin = 0

            #
            #
            # Your Code
            self.msg.linear.x = float(controllerEffortLin)
            self.msg.angular.z = float(controllerEffortRot)
            #
            #
            #
            
        else:
            print("=> No landmarks recognized!")

            #
            #
            # Your Code
            self.msg.linear.x = 0.0
            self.msg.linear.y = 0.0
            self.msg.linear.z = 0.0
            self.msg.angular.x = 0.0
            self.msg.angular.y = 0.0
            self.msg.angular.z = 0.0
            #
            #
            #
            self.integralLin = 0
            self.integralRot = 0
                
            
        #
        #
        # Your Code
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing ang: {self.msg.angular.z}")
        self.get_logger().info(f"Publishing lin: {self.msg.linear.x}")
        #
        #
        #
        cv.imshow("Live View", currImage)
        cv.waitKey(1)
        print("cmd_vel")
    

  
def main(args=None):
    try:
        #
        #
        # YOur Code
        rclpy.init(args=args)
        image_subscriber = ImageSubscriber()
        # Spin the node so the callback function is called continiously.
        rclpy.spin(image_subscriber)
        #
        #
        #
        

    except KeyboardInterrupt as e:
        print("\nEnded with: KeyboardInterrupt")
    except BaseException as e:
        print("Exception")
    
    #
    #
    # Your Code
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
    #
    #
    #
  
if __name__ == '__main__':
  main()