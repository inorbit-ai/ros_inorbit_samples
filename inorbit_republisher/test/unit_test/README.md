# Unit Test for ROS Republisher Node

This repository contains a unit test for a ROS node that republishes messages from `/input_topic` to `/output_topic`. The test ensures the republisher node processes and republishes messages correctly.

## Test Description

The unit test is implemented in `test_republisher.py` and uses the `unittest` framework along with `rostest`. It includes the following test cases:

- **`test_message_republishing`**: 
  - Publishes a test message (`key=value`) to `/input_topic`.
  - Verifies that the republisher node republishes the message with the correct prefix (`input_to_output=`) to `/output_topic`.

- **`test_no_message`**:
  - Ensures that no messages are received on `/output_topic` at the start of the test.

The test subscribes to `/output_topic` and collects received messages for validation.

## Running the Unit Test
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
  . ~/catkin_ws/devel/setup.bash
	rostest inorbit_republisher  test_republisher.test
  ```