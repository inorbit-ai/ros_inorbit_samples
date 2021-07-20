# InOrbit republisher for ROS 1

This directory includes a republisher that allows mapping from arbitrary ROS values to ``InOrbit`` [custom data](https://www.inorbit.ai/faq#publish-custom-data) key/value pairs for application-specific observability.

Currently only mapping from ROS topics is supported. The republisher could be extended to map actions, services and parameters.

## Usage

Create a YAML config file specifying the mappings you would like to use using this format:

```yaml
  republishers:
  - topic: "/fruits_per_cubic_m"
    msg_type: "fruit_msgs/Citrus"
    mappings:
    - field: "num_oranges"
      mapping_type: "single_field"
      out:
        topic: "/inorbit/custom_data/0"
        key: "oranges"
  - topic: "/hardware/status"
    msg_type: "hw_msgs/HardwareStatus"
    mappings:
    - field: "status"
      mapping_options:
        fields: ["lidar", "motor", "battery"]
        filter: 'lambda x: (x.status == 1)'
      mapping_type: "array_of_fields"
      out:
        topic: "/inorbit/custom_data/0"
        key: "hardware_error"
```

Then launch the ``republisher.py`` script passing the config file as the ``config`` param.

A suggested way to organize this is by creating the config file and launch file in your package, then the corresponding launch file would like like this:

```xml
<launch>
  <node name="inorbit_republisher" pkg="inorbit_republisher" type="republisher.py">
    <param name="config" textfile="$(find inorbit_republisher)/config/example.yaml" />
  </node>
</launch>
```

## Mapping types

The republisher can map the ROS values to single field (e.g. ``'fruit=apple'``) or to an array of fields (e.g. ``'fruits=[{fruit1: apple, fruit2: orange}, {fruit1: melon, fruit2: apple}]'``). The former is useful to capture simple fields and the latter to get data from an array of values.

### Array of fields: mapping options

When republishing an array of fields, you can include a set of ``mapping_options`` for each ``mapping``. These include:

* `fields`: a set of fields that you'd like to capture from each array element. For example, if each array element contains the elements ``[a, c, d, e]`` and you'd like to get ``a`` and ``c`` only, you can specify it as:

  ```yaml
  mapping_options:
    fields: ["a", "c"]
  ```

  If no fields are specified, the republisher will get all the fields in each array element.

* `filter`: a lambda expression that can be used to pick certain array elements based on a condition. For example, if you'd like to republish array elements where ``c > 5`` only, you can do it with:

  ```yaml
  mapping_options:
    filter: 'lambda x: (x > 5)'
  ```

## Building and running locally

Find below instructions for building the package and running the node using the the code on the workspace (see also [catkin](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)).

### Build

```bash
cd ~/catkin_ws
rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=noetic
catkin clean
catkin build inorbit_republisher --verbose
```

### Run

```bash
. ~/catkin_ws/install/setup.zsh
# Using the launch file under the 'launch' directory
roslaunch launch/example.launch
```

## TODO

* Mapping of nested, top-level and complex (object) fields
* Schema validation
* Better documentation
* Error handling
* Proper logging
* Latched topic support
* Mapping of ROS parameters
