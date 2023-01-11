---
title: foxy on PC
page_id: foxy_pc
---

# Instructions for ROS2 Foxy on PC

> **_NOTE:_**
> This tutorial is tested on Ubuntu 20.04.5 LTS. 

## WSL2 - skip to Ubuntu if you're running Ubuntu natively

Setup WSL2 on Windows and install a Ubuntu as usual: https://learn.microsoft.com/de-de/windows/wsl/install
> **_NOTE:_**
> Windows 11 is recommended, because it has GUI support, which is mandatory for programs like RViz. Additionally it is possible to run into issues with Hyper-V under Windows 10 when trying to setup the Network Bridge.

Look at your WSL IP-adress by running `ifconfig` in WSL and note that it looks weird and is not in the same subnet as your Windows machine or your Raspberry Pi. 

Enable Hyper-V by pressing the Windows key and type `Turn Windows features on or off`, search for `Hyper-V` and enable it. After that, restart your computer.

After the reboot, search for `Hyper-V Manager` in your start menu and open it. If you don't see it in the start menu, type `mmc` in the start menu and follow these instructions on manually installing the snap in feature via 
https://social.technet.microsoft.com/Forums/en-US/adbec344-39b9-4de0-803a-2351b8eb7cad/no-hyperv-quick-create-option-in-start-menu-after-following-setup-guides?forum=win10itprovirt
Before `Hyper-V Manager` is visible in mmc, you'll probably need to install the `RSAT Tools for Windows 10`: 
https://www.microsoft.com/en-us/download/details.aspx?id=45520

In the left pane, click on the name of your computer. Select `Action` in the top bar and choose `Virtual Switch Manager`. 

Click on `New Virtual Switch` in the left pane and select `External Network` as type. Click on `Create Virtual Switch` and name it `WSLBridge`. Choose the network adapter that is connected to your router. (Choose the only LAN or WLAN (whichever you're using) one, that says nothing about `Virtual ...`, `TeamViewer`, `VPN XYZ` or anything simelar.) Make sure that the Box under VLAN-ID is **NOT** ticked. This may cause issues with the internet connection on windows. Next, click `OK` and close the window.

Open a new Powershell (admin) window and paste the following command: 

    "[wsl2]`nnetworkingMode = bridged`nvmSwitch = WSLBridge" | Out-File $home/.wslconfig

This will create a file called `.wslconfig` in your home directory with the following content, where `WSLBridge` is the name of the virtual switch we jut created in the Hyper-V Manager.

    [wsl2]
    networkingMode = bridged
    vmSwitch = WSLBridge

Now stop your WSL by running `wsl --shutdown` in Powershell and restart your WSL. Check your WSL IP-adress again by running `ifconfig` in WSL and note that it is now in the same subnet as your Windows machine or your Raspberry Pi.

> Source: https://github.com/microsoft/WSL/issues/4150#issuecomment-1018524753
## Ubuntu

> **_NOTE:_**
> based on https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup for ROS2 Foxy

Execute the following commands:
    
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
    sudo chmod 755 ./install_ros2_foxy.sh
    bash ./install_ros2_foxy.sh

Install Gazebo

    sudo apt-get install ros-foxy-gazebo-*

Install Cartographer

    sudo apt install ros-foxy-cartographer
    sudo apt install ros-foxy-cartographer-ros

Install Navigation2

    sudo apt install ros-foxy-navigation2
    sudo apt install ros-foxy-nav2-bringup

Install TurtleBot3 Packages via Debian Packages.

    source ~/.bashrc
    sudo apt install ros-foxy-dynamixel-sdk
    sudo apt install ros-foxy-turtlebot3-msgs
    sudo apt install ros-foxy-turtlebot3

Set the ROS environment for PC.

    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
    echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
    source ~/.bashrc

## Check if Installation was successful

Test it by running the following command on the PC:

    ros2 run turtlebot3_teleop teleop_keyboard

and the following command on your Pi

    ros2 topic echo /cmd_vel

Check if the messages are recieved by the Pi. If it's not working, a common cause is that the firewall is enabled. Disable the firewall on both devices by running

    sudo ufw disable

You can also run the bringup launch file on the Turtlebot to test if it is driving:

    ros2 launch turtlebot3_bringup robot.launch.py

And RViz2 on the PC to see the robot in the simulation:

    rviz2

## To automate installation of your built nodes on startup, write this to the bottom of your `~/.bashrc` file.

    cd ~/AutoSys-TurtlebotRos2/YOURWORKSPACE/
    . install/setup.bash
    cd -

## Running the Visual Follower Project of this Repository:

> **_NOTE:_**
> First, the the Turtlebot's Raspberry Pi takes a Picture with its Pycam and publishes it to a topic. This is done by our Picture Publisher Node.
> 
> The Remote PC subscribes to this topic and calculates angular and linear velocity for the Turtlebot the picture with the Picture Processor Node. In Exchange, this Node then publishes the calculated velocities to cmd_vel.
>
> cmd_vel is subscribed by a built in Node from Turtlebot. It is launched by running the Turtlebot3 bringup Command, mentioned above. Now the Turtlebot drives accordingly.

1. Install the Python Packages for the Picture Processor Node

        pip install opencv-contrib-python==4.6.0.66
        pip install opencv-python==4.6.0.66
        pip install numpy==1.24.0
        pip install protobuf==3.20.0
        pip install mediapipe==TODO VERSION VOM RASPI

2. Build the Picture Processor workspace, you''ll need to fetch the dependencies only once
    
        cd ~/AutoSys-TurtlebotRos2/picproc_ws
        sudo apt install python3-rosdep2 -y
        rosdep update
        rosdep install -i --from-path src --rosdistro foxy -y
        colcon build
        . install/setup.bash

3. Run the Picture Processor Node
   
        ros2 run picproc picproc_node

If something is off it's a good idea to check for differences in your `~/.bashrc` file with ours in  `AutoSys-TurtlebotRos2/misc/`.

## Some additional information about the project
> **_NOTE:_**
> We got the file `PoseEstimationMin.py` by doing the "Advanced Computer Vision with Python - Full Course" from freeCodeCamp on youtube [https://youtu.be/01sAkU_NvOY]. It uses OpenCV and Mediapipe to detect the pose of a person in a frame. We use it to get the landmarks of the foot and kneww for our follower node.

### Implementing stop on code abort

If we stop the Node, the Turtlebot will just continue driving with the last command it got. To get the Turtlebot to stop when the Node gets destroyed or `CTRL+C` is input in the console, we put a try and catch block from start to the call of the `spin` Medhod in `main(args=none)`.
    
    try:
        rclpy.init(args=args)
        image_subscriber = ImageSubscriber()
        # Spin the node so the callback function is called.
        rclpy.spin(image_subscriber)
        

    except KeyboardInterrupt as e:
        print("\nEnded with: KeyboardInterrupt")
    except BaseException as e:
        print("BaseException", repr(e))
        
After that, the stop `cmd_vel` message is created. This should be done on the same layer as `except` and not inside `ecxept` since we it to stop always, not just on `CTRL+C` or Exceptions.

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