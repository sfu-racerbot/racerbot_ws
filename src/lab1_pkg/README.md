# lab1_pkg

## Description
`lab1_pkg` is a ROS 2 package containing two nodes: `talker` and `relay`.
- **Talker**: Publishes `ackermann_msgs::msg::AckermannDriveStamped` messages with configurable speed and steering angle.
- **Relay**: Subscribes to the `drive` topic, multiplies speed and steering angle by 3, updates the timestamp, and publishes on `drive_relay`.

## Run
Make sure the package is built in your ROS 2 workspace:
```
colcon build --packages-select lab1_pkg
source install/setup.bash
```
Run nodes individually or with a launch file:
```
ros2 run lab1_pkg talker
ros2 run lab1_pkg relay
ros2 launch lab1_pkg launch.py
```

## Parameters
- **Talker node parameters**:
    - `v` (double, default 0.0): Speed in m/s
    - `d` (double, default 0.0): Steering angle in radians
    - Example: `ros2 run lab1_pkg talker --ros-args -p v:=2.0 -p d:=0.5`
- **Relay node parameters**: None

## Topics

### Published
- `drive` (Talker)
- `drive_relay` (Relay)

### Subscribed
- `drive` (Relay)
