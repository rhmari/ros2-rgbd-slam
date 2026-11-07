FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    MPLBACKEND=Agg

# RTAB-Map (SLAM), Foxglove bridge (visualization from the host), OpenCV/numpy for the dataset player
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-ros \
        ros-humble-foxglove-bridge \
        ros-humble-tf2-ros \
        python3-pip \
        python3-opencv \
        python3-numpy \
        wget \
    && rm -rf /var/lib/apt/lists/*

# evo for trajectory evaluation. Keep numpy < 2 so the apt OpenCV build keeps working.
RUN pip3 install --no-cache-dir "numpy<2" evo

WORKDIR /ws
COPY scripts /ws/scripts
COPY launch /ws/launch

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
CMD ["bash"]
