# Import the necessary libraries
import rclpy                                                                # Python library for ROS 2
from rclpy.node import Node                                                 # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage                                 # For image-messages
from geometry_msgs.msg import Twist                                         # For velocity-messages
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy    # For Quality of Service
               
import cv2 as cv                                                            # For image processing 
import mediapipe as mp                                                      # For image recognition
from cv_bridge import CvBridge                                              # For conversion form OpenCV to ROS2 messages





class ImageProcesser(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        # Sampletime: Period of picture_Publisher_node
        self.Ts = 0.2
        # Rotation PID-Controller parameters
        self.integralRot = 0
        self.PGainRot = 1.95567563236331
        self.IGainRot = 0.4375250435
        # Translation PID-Controller parameters
        self.PGainLin = 4
        self.IGainLin = 1.0
        self.integralLin = 0
        # for conversion between OpenCV and ROS2 msg
        self.br = CvBridge()         
        # For img classification                                                                
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        # ------------------------------------------------------------------------------------------------------------------------------
        
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        # Define Quality of Service profile
        self.qosProfile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
        # Create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Prevent unused variable warning
        self.publisher_ 
        # Create subscriber       
        self.subscription = self.create_subscription(CompressedImage,'imagePi', self.listener_callback, self.qosProfile)
        # Prevent unused variable warning
        self.subscription 
        # Create Twist-message object
        self.msg = Twist()

        # ------------------------------------------------------------------------------------------------------------------------------
        

    def calcX_is(self,results):
        x_is = -1
        if results.pose_landmarks.landmark[24] and results.pose_landmarks.landmark[23]:
            x_is = (results.pose_landmarks.landmark[23].x+results.pose_landmarks.landmark[24].x)/2
        elif results.pose_landmarks.landmark[24]:
            x_is = results.pose_landmarks.landmark[24].x
        elif results.pose_landmarks.landmark[23]:
            x_is = results.pose_landmarks.landmark[23].x
        else:
            print("Needed landmarks for x_is not recognized!")
        return x_is

    def calcY_distance(self,results):
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
        else:
             print("Needed landmarks for y_distance not recognized!")
        return y_distance

    def PIDRot(self, x_is, currImage):
        if x_is > 0:
            # Compute controller effort for rotationary velocity in z
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
            # Visualisation in image
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
        return controllerEffortRot

    def PIDLin(self, y_distance, currImage):
        if y_distance > 0:
            # Compute controller effort for linear velocity in x
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
        return controllerEffortLin
        


    # Callback is called when an image is received
    def listener_callback(self, data):
        """
        Callback function.
        """
        
        # ------------------------------------------------------------------------------------------------------------------------------

        self.get_logger().info('Receiving Image')   # Display the message on the console

        # ------------------------------------------------------------------------------------------------------------------------------
        
        # Convert OpenCV image to ROS Image message
        currImage = self.br.compressed_imgmsg_to_cv2(data)
        
        # Process image
        results = self.pose.process(currImage)
        middle = currImage.shape[1]/2
        # Draw center line
        cv.line(currImage,(int(middle),0),(int(middle),currImage.shape[0]),(255,0,0),thickness=2)

        # If landmarks recognized
        if results.pose_landmarks:
            # Draw landmarks into image
            self.mpDraw.draw_landmarks(currImage, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
            # calc x_is
            x_is = self.calcX_is(results=results)
            # PID Rotation
            controllerEffortRot =self.PIDRot(x_is,currImage)
            # calc y_distance
            y_distance = self.calcY_distance(results=results)
            # PIDLin 
            controllerEffortLin = self.PIDLin(y_distance,currImage)

            # ---------------------------------------------------------------------------------------------------------------------------

            self.msg.linear.x = float(controllerEffortLin)
            self.msg.angular.z = float(controllerEffortRot)

            # ---------------------------------------------------------------------------------------------------------------------------
            
        else:
            # Reset integators
            self.integralLin = 0
            self.integralRot = 0
            print("No landmarks recognized!")

            # ---------------------------------------------------------------------------------------------------------------------------

            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0

            # --------------------------------------------------------------------------------------------------------------------------


                
            
        # ------------------------------------------------------------------------------------------------------------------------------

        # Publish the message
        self.publisher_.publish(self.msg)
        # Log the relevant velocities 
        self.get_logger().info(f"Publishing ang: {self.msg.angular.z}")
        self.get_logger().info(f"Publishing lin: {self.msg.linear.x}")

        # ------------------------------------------------------------------------------------------------------------------------------

        # Show the processed image
        cv.imshow("Live View", currImage)
        cv.waitKey(1)

    

  
def main(args=None):
    try:
        # ------------------------------------------------------------------------------------------------------------------------------

        # Initialization
        rclpy.init(args=args)
        # Create Image Processer Object
        image_processer = ImageProcesser()
        # Spin the node so the callback function is called continiously.
        rclpy.spin(image_processer)

        # ------------------------------------------------------------------------------------------------------------------------------
        

    except KeyboardInterrupt as e:
        print("\nEnded with: KeyboardInterrupt")
    except BaseException as e:
        print("Exception:", repr(e))
    
    # ----------------------------------------------------------------------------------------------------------------------------------
    
    # Stop the robot
    image_processer.msg.linear.x = 0.0
    image_processer.msg.angular.z = 0.0
    image_processer.publisher_.publish(image_processer.msg)

    # Destroy the node explicitly
    image_processer.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
    # ----------------------------------------------------------------------------------------------------------------------------------
  
if __name__ == '__main__':
  main()