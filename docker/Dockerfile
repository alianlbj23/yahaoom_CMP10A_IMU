# Yahboom CMP10A IMU - ROS 2 Jazzy image (driver + RViz2 + rosbridge + foxglove_bridge)
# Built and published by .github/workflows/docker-image.yml as
#   ghcr.io/alianlbj23/yahaoom_cmp10a_imu:jazzy
# Users pull that image through `python3 imu.py`; it is not built locally.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    LIBGL_ALWAYS_SOFTWARE=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_DISTRO=jazzy \
    WS=/ws

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
        ros-jazzy-rviz-imu-plugin \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-tf2-ros \
        ros-jazzy-ament-copyright \
        ros-jazzy-ament-flake8 \
        ros-jazzy-ament-pep257 \
        python3-serial \
        python3-yaml \
        python3-numpy \
        python3-pytest \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# The image only provides the environment. imu.py mounts the repository's
# wit_ros2_imu/ over ${WS}/src/wit_ros2_imu, and --symlink-install makes the
# installed package point at that directory, so source edits take effect on
# the next start without rebuilding the image. This copy is the fallback used
# when the image is run without the mount.
COPY wit_ros2_imu ${WS}/src/wit_ros2_imu

WORKDIR ${WS}
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --packages-select wit_ros2_imu \
    && rm -rf log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh \
    && mkdir -p /config \
    && printf 'source /opt/ros/jazzy/setup.bash\nsource %s/install/setup.bash\n' "${WS}" \
        >> /root/.bashrc

EXPOSE 9090 8765
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "wit_ros2_imu", "imu.launch.py"]
