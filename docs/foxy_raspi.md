---
title: foxy on Raspberry Pi
page_id: foxy_raspi
---

## Instructions for ROS2 Foxy on Raspberry Pi

> **_NOTE:_**
> This tutorial is tested on Raspberry Pi 4B with 4GB RAM. It is part of the Turtlebot3 waffle.


### Prepare SD Card

##### Download the Foxy image for Raspi4b 

Extract the .img file from the .zip

    https://www.robotis.com/service/download.php?no=2064


##### Download and install Raspberry Pi Imager

    https://www.raspberrypi.org/software/

- advanced options (gear) to flash it preconfigured so it is SSH-ready with information about the WiFi didn't work for us
- select the .img and the SD-Card write the image to the SD-Card


##### Setup the SD-Card

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

> **_NOTE:_**
> The default Turtlebot username is `ubuntu` and th password is `turtlebot`.

##### Boot and connect to the Raspberry Pi

Put the SD-Card in your Raspi and power it on. You can either connect a monitor to find out its IP-adress or use your hotspot / router interface to look at it.
Open a Linux or Windows Terminal initiate a SSH connection via.

    ssh ubuntu@<IPADRESS>


### Setup Ubuntu for ROS2 Foxy on Turtlebot3


##### Update everything preinstalled

Confirm that `export ROS_DOMAIN_ID=30 # Default ROS2 Domain ID for TurtleBot3` is in your bashrc file(usually last line) by:

    nano .bashrc

Update Apt:
    
    sudo apt update

Upgrade all existing packages (A `Could not get lock /var/lib/dpkg/lock` error can occur and can be solved by simply rebooting / powercycling the Raspi):

    sudo apt upgrade

ROS2 should now be working now, check it by running:

    ros2 topic list


##### Install LiDAR Driver for Turtlebot3

Run the following commands. You can paste them all in at once. It'll clone a git repo, install the necessary dependencies and build the driver from source. At the end it'll set the LiDAR version as an environment variable in `.bashrc`.

    cd ~/turtlebot3_ws/src
    git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
    cd ~/turtlebot3_ws/src/turtlebot3 && git pull
    rm -r turtlebot3_cartographer turtlebot3_navigation2
    cd ~/turtlebot3_ws && colcon build --symlink-install
    echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
    source ~/.bashrc


##### Setup OpenCR

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


### Setup Visual Studio Code for Programming

##### Install and Setup VSCode

Install VSCode on your Machine(Windows, Linux, Mac)
    
    https://code.visualstudio.com/
 
and install the following Extensions:
    
    Remote-SSH


##### Connect VSCode with the Raspi via SSH

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


### Taking Pictures with OpenCV

Install PIP3

    sudo apt install python3-pip

Install Open CV

    sudo pip3 install -U opencv-python

Setting up the camera
- If you're using the Raspberry Pi Camera, activate it by adding

        start_x=1

    to ```/boot/firmware/config.txt``` .

-   If you're using any other USB Webcam, install the following dependencies 

        apt-get install ffmpeg libsm6 libxext6  -y

Test it by writing the following code in a python file, e.g. `takePicture.py` and run it with the button on the top right. If there is a dropdown menu: choose `Run Python File`. 

    import cv2 as cv

    cap = cv.VideoCapture(0)
    ret, frame = cap.read()
    cv.imwrite('image.png', frame)

You should now see a new file called 'image.png'. If you're sshed on the Raspi with VSCode, you can just click on it and VSCode will be able to display it.