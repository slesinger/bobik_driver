# bobik_driver
HW Interface for Bobik

Robik Driver is suposed to be run on Bobik's Jetson TK1. There is no dependency on ROS. All communications with main computer running ROS2 is vua ZeroMQ. See bobik_bridge.

This Readme also dewscribes all remaining installation needed for Jetson.

# Wire Connections

## GPIO
(https://elinux.org/Jetson/GPIO#GPIO_on_Jetson_TK1))
```
J3A1 02 GND (for ttyTHS1), to GND level shifter
J3A1 03 1.8v (for ttyTHS1), to LV (low voltage) on level shifter
J3A2 65 TX to Jetson UART1 RXD green towards level shifter (needs 1.8v), /dev/ttyTHS1  
J3A2 68 RX to Jetson UART1 TXD white, needs level shifter to 1.8v)
```

# Build

## Prerequisites

> Jetson OS, L4T 21.3 + [Grinch Kernel](https://github.com/jetsonhacks/installGrinch)

Add this to ```mcedit ~/.bashrc```:
```bash
export LC_CTYPE=en_US.UTF-8
export LC_ALL=en_US.UTF-8
export LD_LIBRARY_PATH=/usr/local/lib
```

```bash
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt update
sudo apt-get install git curl htop mc aptitude libtool autoconf aptitude cmake zlib1g-dev libssl-dev libboost-system1.54-dev
update-rc.d -f apport remove
update-rc.d -f cups remove
update-rc.d -f cups-browsed remove
update-rc.d -f saned remove
service --status-all
sudo nmcli dev wifi connect <YOUR_SSID_HERE> password '<YOUR_KEY_HERE>'
sudo nmcli dev
sudo nmcli connection
```

Setp NTP servers in ```/etc/ntp.conf```:
```
server 0.cz.pool.ntp.org
server 1.cz.pool.ntp.org
server 2.cz.pool.ntp.org
server 3.cz.pool.ntp.org
```

> If wifi connection has slow and unstable ping, try to set ```wifi.powersave = 2``` to disable power management. (```iwconfig wlan0 power off```)

```sudo mcedit /etc/init/rc-sysinit.conf``` change to ```env DEFAULT_RUNLEVEL=3```


```sudo mcedit /etc/sudoers``` add ```ubuntu    ALL=NOPASSWD: ALL```

Get rid of Ubuntu error reports: ```sudo mcedit /etc/default/whoopsie``` change to ```report_crashes=false```. Then ```sudo service whoopsie stop```



## Install gcc-9
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test  #press enter then
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

## Install libzmq 5.x+
```bash
cd
git clone https://github.com/zeromq/libzmq.git
cd libzmq
./autogen.sh
./configure
make
sudo make install
```

## Install bobik_arduino
```bash
cd
git clone https://github.com/slesinger/bobik_arduino.git
```

### Install PlatformIO
```bash
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:fkrull/deadsnakes   #press enter
sudo apt-get update
mkdir ~/python
cd ~/python
wget https://www.python.org/ftp/python/3.6.3/Python-3.6.3.tgz
tar -xvf Python-3.6.3.tgz
cd Python-3.6.3
sudo ./configure --enable-optimizations
make -j8           #if it gets stuck on tests, kill ongoing subprocess to unblock
sudo make install
wget https://bootstrap.pypa.io/pip/3.6/get-pip.py
sudo python3.6 get-pip.py
python3 -m pip install platformio
cd ~/bobik_arduino
```

## Install bobik_driver
```bash
cd
git clone https://github.com/slesinger/bobik_driver.git
cd ~/bobik_driver
rm src/protocol_types.h
ln -s /home/ubuntu/bobik_arduino/lib/protocol/protocol_types.h src/protocol_types.h
mkdir build && cd build && cmake ..
make
sudo make install
```
Binary will install to /usr/local/bin

# Run
```
bobik_driver
```


# Test

## Move
Run on main computer:
```
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Startup scripts

Copy ```startup/bobik.conf``` to ```/etc/init```.
```
sudo initctl reload-configuration
sudo service bobik start
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
