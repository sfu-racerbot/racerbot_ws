# safety_node

## Description
`safety_node` is a ROS 2 package containing the node `safety_node`
- **Safety Node**: Subscribes to `odom` and `scan` for speed and LiDAR measurements, loops through the LiDAR scan data, and determines if we will have an imminent crash. If so, it publishes to `drive` with its speed set to 0 to brake the car.

## Run
Make sure the package is built in your ROS 2 workspace:
```
cd /racerbot_ws
colcon build --packages-select safety_node
source install/setup.bash
```

1. Run nodes with a launch file:
```
ros2 launch safety_node safety_node_launch.py
```
2. Or run the node individually:
```
ros2 run safety_node safety_node
```

## Parameters
- None

## Topics

### Published
- `drive`

### Subscribed
- `odom`
- `scan`
