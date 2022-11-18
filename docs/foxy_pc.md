---
title: foxy on PC
page_id: foxy_raspi
---

# Instructions for ROS2 Foxy on PC

> **_NOTE:_**
> This tutorial is tested on Ubuntu 20.04.5 LTS. 

## WSL2 - skip to Ubuntu if you're running Ubuntu natively

Setup WSL2 on Windows and install a Ubuntu as usual.
> **_NOTE:_**
> Windows 11 is recommended, because it has GUI support, which is mandatory for programs like RViz. Additionally it is possible to run into issues with Hyper-V under Windows 10 when trying to setup the Network Bridge.

Look at your WSL IP-adress by running `ifconfig` in WSL and note that it looks weird and is not in the same subnet as your Windows machine or your Raspberry Pi. 

Enable Hyper-V by pressing the Windows key and type `Turn Windows features on or off`, search for `Hyper-V` and enable it. After that, restart your computer.

After the reboot, search for `Hyper-V Manager` in your start menu and open it. In the left pane, click on the name of your computer. Select `Action` in the top bar and choose `Virtual Switch Manager`. 

Click on `New Virtual Switch` in the left pane and select `External Network` as type. Click on `Create Virtual Switch` and name it `WSLBridge`. Choose the network adapter that is connected to your router. (Choose the only LAN or WLAN (whichever you're using) one, that says nothing about `Virtual ...`, `TeamViewer`, `VPN XYZ` or anything simelar.) Tick the Box under VLAN-ID and click `OK` and close the window.

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
    source ~/.bashrc

