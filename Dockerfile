# Container image for the InOrbit republisher: maps arbitrary ROS 2 topic values
# onto InOrbit custom_data key/values.
#
# Built from this branch's own source rather than a pinned clone, so the image
# always matches the commit it was built from. Branches here are per-distro
# (humble-devel -> humble), which is what picks the base image and the tag.
#
# The build workspace is kept in the image on purpose: ros-base already ships
# colcon and launch_testing, so `colcon test` runs against the published image
# itself rather than a separate test stage that could drift from it.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

# ros-base ships only the default RMW (fastrtps), so this image crash-loops on
# a fleet running any other transport -- the node dies on a missing
# librmw_*.so. Carrying all three keeps it transport-agnostic:
# RMW_IMPLEMENTATION picks one at runtime and the default is unchanged.
# No ldconfig needed -- the entrypoint sources setup.bash, which already puts
# zenoh_cpp_vendor/lib on LD_LIBRARY_PATH.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY inorbit_republisher /ws/src/inorbit_republisher
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws && colcon build --packages-select inorbit_republisher

# Source ROS + the built overlay before the CMD.
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nsource /ws/install/setup.bash\nexec "$@"\n' \
      "${ROS_DISTRO}" > /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# No config is baked in -- mount your mappings at /config/republisher.yaml, or
# override the CMD to point at your own path. See inorbit_republisher/README.md
# for the config format.
CMD ["ros2", "run", "inorbit_republisher", "republisher", "--ros-args", "-p", "config:=/config/republisher.yaml"]
