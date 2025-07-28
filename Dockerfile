FROM ros:humble-ros-base

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo repository  
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y gz-garden && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace
COPY . /workspace/src/dwa_3d_simulation/

# Source ROS2 and build the package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select dwa_3d_simulation"

# Set up entrypoint
RUN echo '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nexec "$@"' > /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]