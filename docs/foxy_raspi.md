---
title: foxy on Raspberry Pi
page_id: foxy_raspi
---

# Instructions for ROS2 Foxy on Raspberry Pi

> **_NOTE:_**
> This tutorial is tested on Raspberry Pi 4B with 4GB RAM. It is part of the Turtlebot3 waffle. 


## Prepare SD Card

> **_NOTE:_**
> based on https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup for ROS2 Foxy


## Download the Foxy image for Raspi4b 

Extract the .img file from the .zip

    https://www.robotis.com/service/download.php?no=2064


## Download and install Raspberry Pi Imager

    https://www.raspberrypi.org/software/

- advanced options (gear) to flash it preconfigured so it is SSH-ready with information about the WiFi didn't work for us
- select the .img and the SD-Card write the image to the SD-Card


## Setup the SD-Card

Connect the SD-Card to a Computer that is able to mount ext4 file systems or connect it to a Linux VM under Windows.

On the SD-Card go to `/etc/netplan/` and edit the `50-cloud-init.yaml` file to include the information about your WiFi Network.

    network:
        ethernets:
            eth0:
                dhcp4: true
                optional: true
        version: 2
        wifis:
            wlan0:
                dhcp4: yes
                dhcp6: yes
                access-points:
                    YOURWIFISSID:
                        password: YOURWIFIPASSWORD

If you're working directly on your pi, use 

    sudo nano /etc/netplan/50-cloud-init.yaml

to edit the file. and then 
    
    sudo netplan apply

to apply the changes.

> **_NOTE:_**
> The default Turtlebot username is `ubuntu` and th password is `turtlebot`.

## Boot and connect to the Raspberry Pi

Put the SD-Card in your Raspi and power it on. You can either connect a monitor to find out its IP-adress or use your hotspot / router interface to look at it.
Open a Linux or Windows Terminal initiate a SSH connection via.

    ssh ubuntu@<IPADRESS>


## Setup Ubuntu for ROS2 Foxy on Turtlebot3


## Update everything preinstalled

Confirm that `export ROS_DOMAIN_ID=30 # Default ROS2 Domain ID for TurtleBot3` is in your bashrc file(usually last line) by:

    nano .bashrc

Update Apt:
    
    sudo apt update

Upgrade all existing packages (A `Could not get lock /var/lib/dpkg/lock` error can occur and can be solved by simply rebooting / powercycling the Raspi):

    sudo apt upgrade

ROS2 should now be working now, check it by running:

    ros2 topic list


## Install LiDAR Driver for Turtlebot3

Run the following commands. You can paste them all in at once. It'll clone a git repo, install the necessary dependencies and build the driver from source. At the end it'll set the LiDAR version as an environment variable in `.bashrc`.

    cd ~/turtlebot3_ws/src
    git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
    cd ~/turtlebot3_ws/src/turtlebot3 && git pull
    rm -r turtlebot3_cartographer turtlebot3_navigation2
    cd ~/turtlebot3_ws && colcon build --symlink-install
    echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
    source ~/.bashrc


## Setup OpenCR

Run the following commands. Again you can paste them all in at once.

    sudo dpkg --add-architecture armhf
    sudo apt update
    sudo apt install libc6:armhf
    export OPENCR_PORT=/dev/ttyACM0
    export OPENCR_MODEL=waffle
    rm -rf ./opencr_update.tar.bz2
    wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
    tar -xjf ./opencr_update.tar.bz2

Upload firmware to the OpenCR.

    cd ~/opencr_update
    ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr


## Setup Visual Studio Code for Programming

## Install and Setup VSCode

Install VSCode on your Machine(Windows, Linux, Mac)
    
    https://code.visualstudio.com/
 
and install the following Extensions:
    
    Remote-SSH


## Connect VSCode with the Raspi via SSH

Open VSCode and click on the green >< in the bottom left corner. 
Select `Remote-SSH: Connect to Host...` and the click on the `+ Add New SSH Host` button.
Enter the SSH Command:

    ssh ubuntu@<IPADRESS>

You've now saved a new SSH connection in your `.ssh\config` file. To connect to the Raspi in the future, just click on the green >< in the bottom left corner, select `Remote-SSH: Connect to Host...` and select the IP-adress from the list.

In the new Window you'll be asked to input your password.

Now again go to your Extensions tab and install the following Extensions. Make sure that VSCode installs them on the Raspi and not on your computer.
    
    Python
    Pylint
    
Go to `File` and click on `Open Folder...`, the default is your user directory: `/home/ubuntu`


## Taking Pictures with OpenCV

Install PIP3

    sudo apt install python3-pip

Install Open CV

    sudo pip3 install -U opencv-python

Setting up the camera
- If you're using the Raspberry Pi Camera, activate it by adding

        start_x=1

    to `/boot/firmware/config.txt` .

-   If you're using any other USB Webcam, install the following dependencies 

        apt-get install ffmpeg libsm6 libxext6  -y

Test it by writing the following code in a python file, e.g. `takePicture.py` and run it with the button on the top right. If there is a dropdown menu: choose `Run Python File`. 

    import cv2 as cv

    cap = cv.VideoCapture(0)
    ret, frame = cap.read()
    cv.imwrite('image.png', frame)

You should now see a new file called `image.png`. If you're sshed on the Raspi with VSCode, you can just click on it and VSCode will be able to display it.


## Create a ROS2 Package and run your first ROS2 Node

> **_NOTE:_**
> based on https://medium.com/schmiedeone/getting-started-with-ros2-part-2-747dd63bdcb


Create a workspace for your ROS2 packages, in this example we created it inside this repository.

    mkdir -p ~/AutoSys-TurtlebotRos2/ros2_ws/src

cd into the workspace and build it.

    cd ~/AutoSys-TurtlebotRos2/ros2_ws
    colcon build

To create a new package, cd into the `src` folder and run the following command. `Talker_listener` is the name of the package.

    cd ~/AutoSys-TurtlebotRos2/ros2_ws/src
    ros2 pkg create --build-type ament_python talker_listener

Create your Node file in `.../src/talker_listener/talker_listener` as talker_node.py. Copy the code from below.
    
    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String


    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

Navigate to `~/AutoSys-TurtlebotRos2/ros2_ws/src/talker_listener/package.xml` and add a line for every dependency you need. In this case we need `rclpy` and `std_msgs`.

    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>

For example if you're using OpenCV, you would add the following line:

    <exec_depend>opencv-python</exec_depend>

To add an entry point, go to `~/AutoSys-TurtlebotRos2/ros2_ws/src/talker_listener/setup.py` and add the following line to the `setup()` function.

    entry_points={
        'console_scripts': [
            'talker_node = talker_listener.talker_node:main',
        ],
    },

If `entry_points` already exists, add the line to the dictionary.

Before we build, it is useful to check `~/AutoSys-TurtlebotRos2/ros2_ws/src/talker_listener/setup.cfg` and make sure that the following lines are uncommented.

    [develop]
    script-dir=$base/lib/talker_listener
    [install]
    install-scripts=$base/lib/talker_listener

Optionally you can check for missing dependencies by rosdep2. First install rosdep2 via

    cd ~/AutoSys-TurtlebotRos2/ros2_ws
    sudo apt install python3-rosdep2 -y
    rosdep update

    rosdep install -i --from-path src --rosdistro foxy -y

Now we can build the package. Make sure you're in the workspace folder.

    cd ~/AutoSys-TurtlebotRos2/ros2_ws
    colcon build

At last: source the terminal by using
    
    . install/setup.bash

and finally run the node with

    ros2 run talker_listener talker_node

check it by running one of the following commands

    ros2 topic list
    ros2 topic echo /topic

## To automate installation of your own code on startup, write this to the bottom of your `~/.bash.rc` file.

    cd ~/AutoSys-TurtlebotRos2/YOURWORKSPACE/
    . install/setup.bash
    cd ~
    # Add shortcut to simplify bringup command
    alias r2b='ros2 launch turtlebot3_bringup robot.launch.py'

## Create Follower Node

Install the following package

    sudo apt install mediapipe

Copy the talker node and rename it to `follower_node.py`

Make your new Node known to `setup.py` by adding this line: `'follower = talker_listener.follower_node:main'` to the `entry_points` dictionary, so it looks like this:

    entry_points={
        'console_scripts': [
                'talker_listener = talker_listener.talker_node:main',
                'follower = talker_listener.follower_node:main'
        ],
    },

# TODO ADD DEPENDENCIES

> **_NOTE:_**
> We got the file `PoseEstimationMin.py` by doing the "Advanced Computer Vision with Python - Full Course" from freeCodeCamp on youtube [https://youtu.be/01sAkU_NvOY]. It uses OpenCV and Mediapipe to detect the pose of a person in a frame. We use it to get the landmarks of the foot and kneww for our follower node.

## Install additional libraries

    pip install mediapipe
    pip install protobuf==3.20.0

## Adding the libraries

Import the libraries for the Computer Vision Part and the ROS2 Node `Twist`, which contains the driving commands for the Turtlebot.

    import cv2 as cv
    import mediapipe as mp
    import time 

    from geometry_msgs.msg import Twist

## Modyfing the Publisher.

In the __init__ function we swap out the datatype `String`with `Twist` and change the topic `topic` to `cmd_vel`.

    self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

Next, we change the Method `timer_callback(self)` to include the logic of our program...

# TODO LOGIC

## Implementing stop on code abort

If we stop the Node, the Turtlebot will just continue driving with the last command it got. To get the Turtlebot to stop when the Node gets destroyed or `CTRL+C` is input in the console, we put a try and catch block from start to the call of the `spin` Medhod in `main(args=none)`.
    
    try:
            rclpy.init(args=args)
            minimal_publisher = MinimalPublisher()
            rclpy.spin(minimal_publisher)
            

        except KeyboardInterrupt as e:
            print("Ended with: KeyboardInterrupt")
        
After that we create another node to publish the stop `cmd_vel` message. This should be done on the same layer as `except` and not inside `ecxept` since we it to stop always, not just on `CTRL+C`.

    #stop regardless of how the node stopped spinning:
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

    minimal_publisher.destroy_node()
    rclpy.shutdown()