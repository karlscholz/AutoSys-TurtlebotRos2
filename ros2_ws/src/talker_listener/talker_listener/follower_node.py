import rclpy
import cv2 as cv
import mediapipe as mp
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    msg = Twist()   

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.switch = 0

    def timer_callback(self):
        #msg = Twist()
        #msg.data = 'Hello World: %d' % self.i
        mpDraw = mp.solutions.drawing_utils
        mpPose = mp.solutions.pose
        pose = mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)


        capture = cv.VideoCapture(0)

        success, img = capture.read()
        imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        results = pose.process(imgRGB)
        if results.pose_landmarks:
            dist_l = 0
            dist_r = 0
            if results.pose_landmarks.landmark[25]:
            #   mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
                dist_l = np.sqrt((results.pose_landmarks.landmark[25].x-results.pose_landmarks.landmark[27].x)**2+ (results.pose_landmarks.landmark[25].y-results.pose_landmarks.landmark[27].y)**2)
            if results.pose_landmarks.landmark[28]:  
                dist_r = np.sqrt((results.pose_landmarks.landmark[26].x-results.pose_landmarks.landmark[26].x)**2+ (results.pose_landmarks.landmark[28].y-results.pose_landmarks.landmark[28].y)**2)
            if dist_l == 0 and dist_r == 0:
                self.msg.linear.x = 0.0
                self.msg.linear.y = 0.0
                self.msg.linear.z = 0.0

                self.msg.angular.x = 0.0
                self.msg.angular.y = 0.0
                self.msg.angular.z = 0.0
            else:
                middle = imgRGB.shape[1]/2
                deadzone = 0.1*imgRGB.shape[1]
                if dist_l >= dist_r:
                    dir = (results.pose_landmarks.landmark[25].x+results.pose_landmarks.landmark[27].x)/2 
                else:
                    dir = (results.pose_landmarks.landmark[26].x+results.pose_landmarks.landmark[28].x)/2  
                if abs(dir-middle) < deadzone:
                    self.msg.angular.x = 0.0
                    self.msg.angular.y = 0.0
                    self.msg.angular.z = 0.0
                elif dir-middle < 0:
                    self.msg.angular.z = 0.5
                    print("Left")
                else:
                    self.msg.angular.z = -0.5
                    print("Right")
        
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing: {self.msg.angular.z}")
     


def main(args=None):
    capture = cv.VideoCapture(0)
    suc, img = capture.read()
    mpDraw = mp.solutions.drawing_utils
    mpPose = mp.solutions.pose
    pose = mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    results = pose.process(img)
    if results.pose_landmarks:
        mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
    cv.imwrite("img_test.jpg",img)

    try:
        rclpy.init(args=args)
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)
        

    except KeyboardInterrupt as e:
        print("Ended with: KeyboardInterrupt")
        print("Stopping \"Follower\" - Node")
        node = rclpy.create_node('stop_follower')
        pub = node.create_publisher(Twist, 'cmd_vel', 10)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
    #finally:
        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()