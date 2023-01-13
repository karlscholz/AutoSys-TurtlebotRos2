import rclpy                                                                                    # Python library for ROS 2
from rclpy.node import Node                                                                     # Handles the creation of nodes
from geometry_msgs.msg import Twist                                                             # For velocity-messages
               
class myClass(Node):                                                                            # my Class
    def __init__(self):                                                                         # Constructor for my Class
        self.variable = 1.5                                                                     # remembers stuff Objectwide
        self.msg = Twist()                                                                      # variable to publish for driving
        super().__init__('cha_cha_real_smooth')                                                 # initiates Node with Name
        self.publisher_ = self.create_publisher(msg_type=Twist, topic='cmd_vel', qos_profile=10)# creates publisher for cmd_vel
        self.timer = self.create_timer(timer_period_sec=2, callback=self.timer_callback)        # timer, calls callback every 2sec
 
    def timer_callback(self):                                                                   # method that gets called by timer
        self.variable = [1.5, -1.5][self.variable==1.5]                                         # switches self.variable +-1.5
        self.msg.angular.z = self.variable                                                      # sets angular.z to self.variable
        self.publisher_.publish(self.msg)                                                       # publishes variable msg
        self.get_logger().info(f"Publishing ang: {self.msg.angular.z}")                         # logs/prints what was published
 
def main(args=None):                                                                            # function main
    try:                                                                                        # catches errors
        rclpy.init(args=args)                                                                   # Initialization of RCL
        myObject = myClass()                                                                    # Create myObject of myClass
        rclpy.spin(myObject)                                                                    # Starts running the Node
    except KeyboardInterrupt as e:                                                              # Catches Ctrl+C
        print("\nEnded with: KeyboardInterrupt: ", repr(e))                                     # prints Why it ended
    except BaseException as e:                                                                  # Catches all other Exceptions
        print("Exception: ", repr(e))                                                           # prints the Exception Info

    myObject.msg.angular.z = 0.0                                                                # makes one last stop msg
    myObject.publisher_.publish(myObject.msg)                                                   # publishes the Twist msg to stop
    myObject.get_logger().info(f"Publishing ang: {myObject.msg.angular.z}")                     # logs/prints what was published

    myObject.destroy_node()                                                                     # Destroys the node explicitly
    rclpy.shutdown()                                                                            # Shutdown the ROS client library

if __name__ == '__main__':                                                                      # checks if file is run top level
  main()                                                                                        # calls main and starts everything