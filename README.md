# bobik_driver
HW Interface for Bobik

Robik Driver is suposed to be run on Bobik's Jetson TK1. There is no dependency on ROS. All communications with main computer running ROS2 is vua ZeroMQ. See bobik_bridge.

This Readme also dewscribes all remaining installation needed for Jetson.

# Build

## Prerequisites

> Jetson OS, L4T 21.8
```bash
export LC_CTYPE=en_US.UTF-8
export LC_ALL=en_US.UTF-8
sudo aptitude install git curl htop mc aptitude libtool autoconf
```

Install gcc-9
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 20
sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
sudo update-alternatives --set cc /usr/bin/gcc
sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
sudo update-alternatives --set c++ /usr/bin/g++
```


> Install libzmq 5.x+
```bash
git clone https://github.com/zeromq/libzmq.git
cd libzmq
./autogen.sh
./configure
make
sudo make install
```

Assuming that git repo is checkout out to ```~/bobik_driver``` inspite it is not a ros package.

```bash
cd ~/bobik_driver
mkdir build
cd build
cmake ..
make
sudo make install
```
Binary will install to /usr/local/bin

# Run
```
bobik_driver
killall bobik_driver
```


# Test

## Move
```
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
# Jetson TK1
https://distrustsimplicity.net/articles/nvidia-jetson-tk1-wifi/

# Raspberry PI Installation

## Instal raspi-config on Ubuntu
https://raspberrypi.stackexchange.com/questions/111728/how-to-get-raspi-config-on-ubuntu-20-04

## Startup scripts

Copy ```launch/bobik.service``` to ```/etc/systemd/system```.
```
sudo systemctl enable bobik
sudo systemctl start bobik
```

## Install XV11 Lidar Driver
```
sudo usermod -a -G tty ubuntu
sudo aptitude libasio-dev
cd ~/ros2_foxy/src
git clone https://github.com/mjstn/xv_11_driver
cd ~/ros2_foxy
colcon build
. ~/ros2_foxy/install/local_setup.bash
ros2 run xv_11_driver xv_11_driver --ros-args -p port=/dev/ttyS0
```
