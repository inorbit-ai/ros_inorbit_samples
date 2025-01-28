# Sample data for testing

A very simple ``rosbag`` with hardcoded data for smoke testing the transformer node.

```bash
$ rosbag info fake_sensors_sample.bag
path:        fake_sensors_sample.bag
version:     2.0
duration:    18.8s
start:       Jul 19 2021 15:55:36.25 (1626720936.25)
end:         Jul 19 2021 15:55:55.04 (1626720955.04)
size:        14.2 KB
messages:    20
compression: none [1/1 chunks]
types:       sensor_msgs/MagneticField [2f3b0b43eed0c9501de0fa3ff89a45aa]
             sensor_msgs/Temperature   [ff71b307acdbe7c871a5a6d7ed359100]
topics:      /my_magnetic_field   10 msgs    : sensor_msgs/MagneticField
             /my_temperature      10 msgs    : sensor_msgs/Temperature
```

To validate the node works launch the sample by using the ``sample_data.launch`` launch file and look at ``out`` topic: ``rostopic echo /inorbit/custom_data/0``.
1. **Start ROS2 docker container (optional)**:
You can run the commands below for building and running the republisher inside a docker container.
  ```bash
  docker run -ti --rm \
    --workdir /root/catkin_ws/ \
    -v .:/root/catkin_ws/src/inorbit_republisher \
    osrf/ros:noetic-desktop
  # Install catkin
  apt update && apt install python3-catkin-tools python3-osrf-pycommon -y
  ```
2. **Build the Workspace**:
  Ensure the workspace is built and the environment is sourced:
  ```bash
  cd ~/catkin_ws
  rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=noetic
  catkin clean 
  catkin build inorbit_republisher --verbose
  ```
3. **Source and Run the Workspace**:
  ```bash
  . ~/catkin_ws/install/setup.zsh
  roslaunch sample_data.launch
# On a different terminal windows
$ rostopic echo /inorbit/custom_data/0
data: "my_temperature=41.0"
---
data: "my_magnetic_field_x=1.4"
---
data: "my_magnetic_field_y=0.2"
---
data: "my_magnetic_field_z=0.6"
---
data: "my_temperature=41.0"
...
```