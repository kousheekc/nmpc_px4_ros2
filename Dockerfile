FROM osrf/ros:humble-desktop-full-jammy

# Set the working directory
WORKDIR /workspace

# Install required dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Clone PX4-Autopilot, modify dd_topics.yaml with custom_dds_topics.yaml run its setup script
RUN git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.15.2 --recursive
COPY 3rd_party/custom_dds_topics.yaml PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Clone and build Micro XRCE-DDS Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig /usr/local/lib/

# Copy the package to the container
COPY ./ /workspace/nmpc_px4_ros2_ws/src

# Build ACADOS
RUN cd /workspace/nmpc_px4_ros2_ws/src/3rd_party/acados && mkdir build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make install -j4

# Install ACADOS python interface
RUN pip install virtualenv && \
    virtualenv env --python=/usr/bin/python3 && \
    /bin/bash -c "source env/bin/activate && pip install -e /workspace/nmpc_px4_ros2_ws/src/3rd_party/acados/interfaces/acados_template"

CMD ["/bin/bash"]