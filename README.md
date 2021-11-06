# bobik_driver
HW Interface for Bobik

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

# Test

## Move
```
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
