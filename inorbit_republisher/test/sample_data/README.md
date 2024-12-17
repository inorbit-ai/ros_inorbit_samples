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

```bash
. ~/catkin_ws/devel/setup.zsh
cd ~/catkin_ws/src/inorbit_republisher/test/sample_data
roslaunch sample_data.launch
# On a different terminal windows
source /opt/ros/noetic/setup.bash
$ rostopic echo my_temperature
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
