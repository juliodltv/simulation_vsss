# simulation_vsss

It's necessary to run the next line command:
>sudo apt-get install ros-noetic-velocity-controllers

https://github.com/lucasw/image_to_v4l2loopback

- Create a virtual camera
> sudo modprobe v4l2loopback exclusive_caps=1 video_nr=1 card_label="Fake"
- Run the node
> rosrun image_to_v4l2loopback stream _device:=/dev/video1 _width:=640 _height:=480 _fourcc:=YV12 image:=/my_camera/image
- Close virtual camera
>  sudo rmmod v4l2loopback
