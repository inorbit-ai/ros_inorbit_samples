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
1. **Start ROS 2 docker container (optional)**:
You can run the commands below for building and running the republisher inside a docker container.
  ```bash
  docker run -ti --rm \
  --workdir /root/ros2_ws/ \
  -v .:/root/ros2_ws/src/inorbit_republisher \
  osrf/ros:foxy-desktop
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
  source install/local_setup.bash
  colcon test --packages-select inorbit_republisher  --event-handlers console_cohesion+ console_direct+
  ```