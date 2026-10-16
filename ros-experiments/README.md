# Exporting ego car's position and velocity from MATLAB ROS2 Node (Publisher) to C++ ROS2 Node (Subscriber) 

Streams ego-vehicle position and velocity from a MATLAB driving simulation into a ROS 2 (Jazzy) C++ node.

## Flow

MATLAB and ROS 2 connect over DDS, not files. They only need the same topic (`/ego_odom`), message type (`nav_msgs/Odometry`), and `ROS_DOMAIN_ID`.

## Components

- **Publisher** - MATLAB script; creates a ROS 2 node and publishes ego car's position and velocity each simulation step.
- **Subscriber** - `ros2_ws/src/first_pkg/src/EgoStateListener.cpp`; subscribes to `/ego_odom`, logs position and velocity.

## Build

```bash
cd ./ros-experiments/ros2_ws/
rm -rf build install log          # only if switching envs (i.e., dev container to WSL)
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run

Terminal 1 - start the subscriber (waits for messages):

```bash
ros2 run first_pkg ego_state_listener
```

Then run the MATLAB script. Expected output in Terminal 1:

```
[INFO] [1792162198.014667983] [ego_state_listener]: [map] pos = [50.250, -1.805, 0.000]  vel = [24.995, -0.476, 0.000]
```

Stop with `Ctrl+C`.

## Verify

Terminal 2 (any directory) - inspect the topic while MATLAB is running:

```bash
ros2 topic list            # confirm /ego_odom appears while MATLAB is running
ros2 topic echo /ego_odom  # print the live message data
ros2 topic hz /ego_odom    # measure the publish rate
```
