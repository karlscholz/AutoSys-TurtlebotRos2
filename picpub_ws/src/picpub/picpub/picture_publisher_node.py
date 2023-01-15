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
        self.stop_threads = False

    def run(self):
        while True:
            ret, self.last_frame = self.camera.read()
            if self.stop_threads:
                break

# picture publisher node as subclass of Node
class PicturePublisher(Node):  
    def __init__(self):
        # -------------------------------------------------------------------------------------------------------------------
        # call constructor
        
        # create capture object
        self.cap = 
        # create qos-profile
        self.qos_Profile = 
        # create publisher 
        self.publisher = 
        # set cycle time
        timer_period =   # seconds
        # create callback call
        self.timer = 
        # --------------------------------------------------------------------------------------------------------------------
        
        self.cam_cleaner = CameraBufferFlusherThread(self.cap)    # saves the images of the raspi cam
        self.counter = 0   
        self.bridge = CvBridge()
        while self.cam_cleaner.last_frame is None:
            pass
        

    def timer_callback(self):
        # -------------------------------------------------------------------------------------------------------------------
        # get last image
        cv_image = 
        # -------------------------------------------------------------------------------------------------------------------
        
        cv.resize(cv_image,(int(cv_image.shape[1]/4),int(cv_image.shape[0]/4)),interpolation=cv.INTER_LINEAR)     # make image smaller
        cv.putText(cv_image, f'{self.counter}',(50,50), cv.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), thickness=2)        # add image counter to the top left corner
        print(self.counter)       # print image counter
       
        # -------------------------------------------------------------------------------------------------------------------
        # publish image
        
        # log publishing
       
        # -------------------------------------------------------------------------------------------------------------------
       
        self.counter += 1         # update picture counter
        cv.waitKey(1)

def main(args=None):
    try:
        # -------------------------------------------------------------------------------------------------------------------
        # initialize ROS

        # create picture publisher node
        picture_publisher = 
        # execute picture publisher node until it is shut down

        # -------------------------------------------------------------------------------------------------------------------

    except KeyboardInterrupt as e:
        print("\nEnded with: KeyboardInterrupt: ", repr(e))
        picture_publisher.cam_cleaner.stop_threads = True
    except BaseException as e:
        print("Exception:", repr(e))
        picture_publisher.cam_cleaner.stop_threads = True
    # -------------------------------------------------------------------------------------------------------------------
    # destroy picture publisher node if it stops running

    # shutdown ROS
    
    # -------------------------------------------------------------------------------------------------------------------
    

if __name__ == '__main__':
   main()