import rclpy
import cv2 as cv
import mediapipe as mp
from rclpy.node import Node
import time 

from std_msgs.msg import String
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
        liveViewScale = 2
        preTime = 0

        success, img = capture.read()
        cv.imwrite('TestPose.jpg',img)
        imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        results = pose.process(imgRGB)
        #print(results.pose_landmarks)
        if results.pose_landmarks:
            mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

        curTime = time.time()
        fps = 1/(curTime-preTime)
        preTime = curTime


        cv.putText(img, str(int(fps)), (10,30), cv.FONT_HERSHEY_COMPLEX, 1.0, (255,255,0), 1)
        img = cv.resize(img, (img.shape[1] * liveViewScale,img.shape[0] * liveViewScale), interpolation=cv.INTER_AREA)

    
        self.msg.angular.z = float(self.i)
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing: {self.msg.angular.z}")
        if self.i >= 0.5:
            self.switch = 0
        if self.i <= -0.5:
            self.switch = 1
        if self.switch == 1:
            self.i += 0.1
        elif self.switch == 0:
            self.i -= 0.1


def main(args=None):
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()