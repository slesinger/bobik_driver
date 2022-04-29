# bobik_driver
HW Interface for Bobik

This Readme also dewscribes all remaining installation needed for RPI.

# Build
```
cd ~/ros2_foxy
colcon build --packages-select bobik_driver
```

# Run
```
ros2 run bobik_driver bobik_driver
killall bobik_driver
```

# ROS2 CLI Tools

eProsima DDS filters topics to individual clients for only those a specific node needs. It is contraproductive for CLI tools like ```ros2 topic list```. CLI tools needs to run as a [SUPER_CLIENT](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#daemon-s-related-commands)

```
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/honza/ros2_foxy/super_client_configuration_file.xml
export ROS_DISCOVERY_SERVER="192.168.1.2"
fastdds discovery -i 0
ros2 daemon stop
ros2 daemon start

ros2 run rviz2 rviz2
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
