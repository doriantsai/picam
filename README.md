# picam
Code to control Raspbery Pi Cameras, configure their settings and capture images over a specified time interval

- Raspbery Pi OS, version 10 (buster)
- enable legacy camera stack
- Python 3.7.3
- pip3 install picamera (should be v1.13)

- Use raspberry Pi Imager for Raspbian OS Buster (legacy), because this has documented ROS Noetic support, while Bullseye does not. Consider migrating to Ubuntu 20.04 with picamera2, which is soon to be supported.
- `sudo raspi-config` to enable ssh and legacy camera stack
- Increase swap RAM to 1GB for faster compiles:

      sudo dphys-swapfile swapoff
	    sudo vim /etc/dphys-swapfile
      
- Change 100 MB to 1024 MB

      sudo dphys-swapfile setup
      sudo dphys-swapfile swapon
      
- Install ROS Noetic directly (from Github Repos) because a number of ROS packages like RVIZ are not available for Debian Buster, and fail to compile from source. Most relevant packages do however still compile. We roughly follow the instructions from this tutorial (https://varhowto.com/install-ros-noetic-raspberry-pi-4/)  with the addition of relevant ROS image packages.

      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list' 
      sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      sudo apt update
      sudo apt-get install -y python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall build-essential cmake
      sudo rosdep init && rosdep update
      mkdir ~/ros_catkin_ws && cd ~/ros_catkin_ws
      rosinstall_generator ros_comm common_msgs image_common image_pipeline vision_opencv vision_msgs --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-vision-wet.rosinstall
      wstool init src noetic-ros_comm-vision-wet.rosinstall
      rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
      
- Compile ROS Noetic (this step can take several hours):

      sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
