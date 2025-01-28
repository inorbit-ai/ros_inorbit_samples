# Sample data for testing

A very simple ``rosbag``  with hardcoded data for smoke testing the transformer node.

```bash
$root@7c94d2cf9659:~# ros2 bag info ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3

closing.
[INFO] [1734553121.938063653] [rosbag2_storage]: Opened database 'ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3' for READ_ONLY.

Files:             ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3
Bag size:          24.0 KiB
Storage id:        sqlite3
Duration:          16.005861016s
Start:             Dec 13 2024 19:45:00.085674077 (1734119100.085674077)
End:               Dec 13 2024 19:45:16.091535093 (1734119116.091535093)
Messages:          34
Topic information: Topic: /my_magnetic_field | Type: sensor_msgs/msg/MagneticField | Count: 17 | Serialization Format: cdr
                   Topic: /my_temperature | Type: sensor_msgs/msg/Temperature | Count: 17 | Serialization Format: cdr

```

To validate the node works launch the sample by using the ``sample_data.launch.xml`` launch file and look at ``out`` topic: ``ros2 topic echo my_temperature``.
1. **Start ROS2 docker container (optional)**:
You can run the commands below for building and running the republisher inside a docker container.
  ```bash
  docker run -ti --rm \
  --workdir /root/ros2_ws/ \
  -v .:/root/ros2_ws/src/inorbit_republisher \
  osrf/ros:humble-desktop
  ```
2. **Build the Workspace**:
  Ensure the workspace is built and the environment is sourced:
  ```bash
  cd ~/ros2_ws
  rosdep install --from-paths src -y --ignore-src
  colcon build --packages-select inorbit_republisher --symlink-install
  ```
3. **Source and Run the Workspace**:
```bash
cd ros2_ws/src
colcon build --packages-select inorbit_republisher --symlink-install
source install/setup.bash
cd
ros2 launch inorbit_republisher sample_data.launch.xml
# On a different terminal windows
source /opt/ros/humble/setup.bash
ros2 topic echo inorbit/custom_data
data: my_magnetic_field_x=1.233536973711507
---
data: my_magnetic_field_y=6.557449696128689
---
data: my_magnetic_field_z=-1.6201375987982995
---
data: my_temperature=29.562268283427592
---
data: my_magnetic_field_x=-3.071746389301242
---
data: my_magnetic_field_y=6.080682955949868
---
data: my_magnetic_field_z=3.952488443512035
---
data: my_temperature=21.159168808582983
---
```