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
        
        # call constructor

        # Define Quality of Service profile
        self.qosProfile = 
        # Create publisher
        self.publisher_ = 
        # Create subscriber       
        self.subscription = 
        # Create Twist-message object
        self.msg = 

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

    def PIDRot(self, x_is):
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
        else:
            # if no x_is can be determined, set controllereffort to 0 and reset the integrator
            controllerEffortRot = 0
            self.integralRot = 0
        return controllerEffortRot

    def PIDLin(self, y_distance):
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

        # logging 
          

        # ------------------------------------------------------------------------------------------------------------------------------
        
        # Convert OpenCV image to ROS Image message
        currImage = self.br.compressed_imgmsg_to_cv2(data)
        
        # Process image
        results = self.pose.process(currImage)

        # If landmarks recognized
        if results.pose_landmarks:
            # Draw landmarks into image
            self.mpDraw.draw_landmarks(currImage, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
            # calc x_is
            x_is = self.calcX_is(results=results)
            # PID Rotation
            controllerEffortRot =self.PIDRot(x_is)
            # calc y_distance
            y_distance = self.calcY_distance(results=results)
            # PIDLin 
            controllerEffortLin = self.PIDLin(y_distance)

            # ---------------------------------------------------------------------------------------------------------------------------

            # Set linear x and angular z velocity: case landmarks recognized 
            

            # ---------------------------------------------------------------------------------------------------------------------------
            
        else:
            # Reset integators
            self.integralLin = 0
            self.integralRot = 0
            print("No landmarks recognized!")

            # ---------------------------------------------------------------------------------------------------------------------------

            # Set linear x and angular z velocity: case landmarks not recognized 


            # --------------------------------------------------------------------------------------------------------------------------


                
            
        # ------------------------------------------------------------------------------------------------------------------------------

        # Publish the message
        
        # Log the relevant velocities 
        


        # ------------------------------------------------------------------------------------------------------------------------------

        # Draw the visualisation of the controller effort / driving commands
        height, width = currImage.shape[:2]
        # Steering, horizontal bar on the bottom of the image
        cv.rectangle(currImage, (50, height-30), (width-50, height - 10), (0, 255, 0), 3)
        cv.rectangle(currImage, (int(width/2), height-30), (int((width/2)-(width/2 -50)*self.msg.angular.z/1.82), height -10), (0, 255, 0), cv.FILLED)
        # Throttle, vertical bar on the right of the image
        cv.rectangle(currImage, (width -10, 50), (width-30, height - 50), (0, 0, 255), 3)
        cv.rectangle(currImage, (width -10, int(height/2)), (width-30,int((height/2)-(height/2 -50)*self.msg.linear.x/0.26)), (0, 0, 255), cv.FILLED)
        # Text
        cv.putText(currImage, f'Steering: {float("{:.2f}".format(self.msg.angular.z))} ', (50, height-50), cv.FONT_HERSHEY_COMPLEX, .75, (0, 255, 0), 2)
        cv.putText(currImage, f'                            Throttle: {float("{:.2f}".format(self.msg.linear.x))} ', (50, height-50), cv.FONT_HERSHEY_COMPLEX, .75, (0, 0, 255), 2)
        
        # Show the processed image
        cv.imshow("Live View", currImage)
        cv.waitKey(1)

    

  
def main(args=None):
    try:
        # ------------------------------------------------------------------------------------------------------------------------------

        # Initialization
        
        # Create Image Processer Object
        
        # Spin the node so the callback function is called continiously.
        

        # ------------------------------------------------------------------------------------------------------------------------------
        

    except KeyboardInterrupt as e:
        print("\nEnded with: KeyboardInterrupt")
    except BaseException as e:
        print("Exception: ", repr(e))
    
    # ----------------------------------------------------------------------------------------------------------------------------------
    
    # Stop the Turtlebot
    



    # Destroy the node explicitly
    
    
    # Shutdown the ROS client library for Python
    
    
    # ----------------------------------------------------------------------------------------------------------------------------------
  
if __name__ == '__main__':
  main()