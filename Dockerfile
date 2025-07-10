# Use the official ROS 2 Humble base image with Gazebo Fortress pre-installed
FROM osrf/ros:humble-desktop-full

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install additional dependencies if needed
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    python3-vcstool \
    python3-rosdep \
    ignition-fortress \
    vim \
    nano \
    openjdk-21-jdk \
    ros-dev-tools \
    python3-colcon-common-extensions \
    wget \
    unzip \
    zip \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENV JAVA_HOME="/usr/lib/jvm/java-21-openjdk-amd64"
ENV PATH="$JAVA_HOME/bin:$PATH"

RUN pip install -U git+https://github.com/colcon/colcon-gradle
RUN pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle
RUN pip install masced_bandits

RUN curl -s https://raw.githubusercontent.com/kas-lab/ros_typedb/590c09cf7a3d2bfbf463c7e0d9367805a2376656/.github/workflows/install_typedb.sh | bash



RUN curl -s "https://get.sdkman.io" | bash \
    && bash -c "source /root/.sdkman/bin/sdkman-init.sh && sdk install gradle 8.5"
    
ENV SDKMAN_DIR="/root/.sdkman"
ENV PATH="$SDKMAN_DIR/bin:$SDKMAN_DIR/candidates/gradle/current/bin:$PATH"   
ENV REBET_TERMINAL_PREFIX=""
# Create workspace directories
RUN mkdir -p /rebetmc_ws/src

# Copy dependencies.repos into the container
COPY rebetmirte.rosinstall /rebetmc_ws/

WORKDIR /rebetmc_ws

# Use vcs to import and clone all packages
RUN vcs import /rebetmc_ws/src < /rebetmc_ws/rebetmirte.rosinstall
RUN touch /rebetmc_ws/src/mirte-ros-packages/mirte_telemetrix_cpp/COLCON_IGNORE

RUN curl -skL https://raw.githubusercontent.com/EGAlberts/ros2_java/refs/heads/param_problem/ros2_java_desktop.repos | vcs import src
RUN git clone https://github.com/EGAlberts/ament_gradle_plugin.git --branch ivanpauno/gradle7.5-compatibility ./src/ament_gradle_plugin
RUN cd /rebetmc_ws/src/ament_gradle_plugin && gradle publishToMavenLocal
# Initialize rosdep
# RUN rosdep update && apt update

# Source ROS 2 setup script, install dependencies, and build the workspace
RUN touch /rebetmc_ws/src/ros2-java/ros2_java_examples/COLCON_IGNORE

# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     rosdep install --from-paths src --ignore-src -r -y"

RUN touch /rebetmc_ws/src/mirte-ros-packages/mirte_telemetrix_cpp/COLCON_IGNORE

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/"]



RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"
    
RUN cd /rebetmc_ws/src/rebet_java && gradle copyDependencies

# RUN rm -rf /var/lib/apt/lists/*

# Set the default entrypoint
ENTRYPOINT ["/bin/bash"]
