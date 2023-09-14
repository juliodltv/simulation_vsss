# simulation_vsss

This repository contains the code to simulate the LARC VSSS Competition using ROS and Gazebo. The code was tested on Ubuntu 20.04 and ROS Noetic.

## Dependencies
Robot Operating System (ROS) on the Noetic version is used to run the code. ROS serves as the framework for the simulation and the [code](https://github.com/Xpsp/larac_vsss) used to control the robots. The installation instructions for ROS Noetic can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

The robots use the [velocity_controllers](http://wiki.ros.org/velocity_controllers) package to control the robot's movement. The package can be installed using the following command:

    sudo apt-get install ros-noetic-velocity-controllers

<div align="center" style="font-size: 18px; margin-bottom: 20px;">The simulator doesn't need more dependencies</div>

However trying to improve the response of the [code](https://github.com/Xpsp/larac_vsss) on the real robots, we load the camera input directly from /dev/videoX, so to get the camera input from the simulator we use the [image_to_v4l2loopback](https://github.com/lucasw/image_to_v4l2loopback) to convert the image topic to a virtual camera.

The flowchart is shown below:
- Run simulator
- Create a virtual camera
- Stream the image topic to the virtual camera
- Run the [code](https://github.com/Xpsp/larac_vsss)

## Instalation
In order create the virtual camera, we use the [v4l2loopback](https://github.com/umlaeute/v4l2loopback) kernel module. To install it, follow the instructions below:

```bash
git clone https://github.com/umlaeute/v4l2loopback
cd v4l2loopback
make
```

To install image_to_v4l2loopback, follow the instructions below:

```bash
cd <path_to_catkin_ws>/src
git clone https://github.com/lucasw/image_to_v4l2loopback
cd ..
catkin_make
```

The following is to create a virtual camera:

    sudo modprobe v4l2loopback exclusive_caps=1 video_nr=2 card_label="Fake"

If you don't get any error, the virtual camera was created successfully. To check if the camera was created, run the following command:

    v4l2-ctl --list-devices

You should see a "Fake" camera in the list. If you get an error, it's so likely that you need to disable the secure boot. To do that, follow the instructions below:
    
    sudo apt install mokutil
    sudo mokutil --disable-validation

When you run the command above, you will be asked to create a temporaly password. After that you need to reboot the computer, when the computer is booting, you will see a blue screen asking you to disable the secure boot, sometimes only an specific character of your password is required, not the whole password.

***READ THE INSTRUCTIONS ABOVE BEFORE REBOOTING THE COMPUTER***

    sudo reboot

After rebooting the computer, try again to create the virtual camera.

## Running the simulation
Follow the previous flowchart the commands are shown below:

- Run simulator
  
        roslaunch simulation_vsss simulation_match.launch
  
- Create a virtual camera
  
        sudo modprobe v4l2loopback exclusive_caps=1 video_nr=2 card_label="Fake"

- Stream the image topic to the virtual camera

        rosrun image_to_v4l2loopback stream _device:=/dev/video2 _width:=800 _height:=600 _fourcc:=YUYV image:=/camera/image_raw

The virtual camera is destroyed when your turn off the computer, alternatively you can run the following command to destroy the virtual camera:

    sudo rmmod v4l2loopback
