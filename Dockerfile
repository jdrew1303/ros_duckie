# Use a specific ROS version for reproducibility
FROM arm32v7/ros:kinetic-ros-base-xenial

# Set environment variables
ENV TERM "xterm"
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO kinetic
ENV LD_LIBRARY_PATH /opt/vc/lib

# Install ROS packages and Python dependencies
RUN apt-get update && apt-get install -y \
    ros-kinetic-perception \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy \
    ros-kinetic-ackermann-msgs \
    python-pip \
    python-smbus \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace and copy only the requirements file first for better caching
WORKDIR /home/ros_bot
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the source code
COPY . .

# Build the catkin workspace
# Use a specific number of jobs to avoid resource exhaustion
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j2"

# Set up the entrypoint to source the workspace
ENTRYPOINT ["/bin/bash", "-c", "source /home/ros_bot/devel/setup.bash && roslaunch launch/main.launch"]