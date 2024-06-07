FROM darpaansr.azurecr.io/ros-cuda:latest
WORKDIR /home/performer/dev_ws
# Install requirements here:
# For example, to install the geometry-msgs package
# RUN apt-get update && apt-get install -y ros-humble-geometry-msgs
# Generally, putting these requirements higher in the file allows you to cache the results and reduce build times!

# Switch to the root user so we can install all dependencies.
USER 0:0
RUN apt-get update && apt-get -y upgrade
RUN apt install -y python3-pip gedit ros-humble-cv-bridge ros-humble-vision-opencv
RUN apt install -y ros-humble-tf-transformations
RUN apt-get update -y --fix-missing
RUN apt install -y ros-humble-rviz2

# Install python dependencies.
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install torch torchvision transformers[torch] Pillow opencv-python"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install scipy matplotlib==3.7.3 tikzplotlib gym gurobipy==11.0.0"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install gym-minigrid==1.0.3 torch==2.1.0 stable-baselines3 gym-unity cvxpy"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install mlagents mlagents-envs"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install numpy==1.22.4"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install shimmy>=0.2.1"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install transforms3d"]
RUN ["/bin/bash", "-c", "pip3 --default-timeout=200 install shapely"]

# Add a custom entrypoint - this overrides the one in the ros-cuda image!
COPY --chown=performer ./docker/ros_entrypoint.sh /ros_entrypoint.sh
# Add a default command - this is a script that should launch your nodes.
COPY --chown=performer ./docker/run_node.sh /run_node.sh
RUN chmod +x /ros_entrypoint.sh
RUN chmod +x /run_node.sh

# Copy the latest security keys to the container and set the security variables:
COPY ./ta3-documentation/sros2/airsim_keystore /airsim_keystore
ENV ROS_SECURITY_ENCLAVE_OVERRIDE=/airsim/adk_key
ENV ROS_SECURITY_KEYSTORE=/airsim_keystore
ENV ROS_SECURITY_ENABLE=false
ENV ROS_SECURITY_STRATEGY=Enforce
ENV MAP_ALTITUDE=15


# Copy the ADK interfaces to the ROS workspace
COPY ./ta3-documentation/ros2/adk_interface /home/performer/dev_ws/src/adk_interfaces
COPY ./ta3-documentation/ros2/airsim_interfaces /home/performer/dev_ws/src/airsim_interfaces

# Switch back to the performer user. I have run into issues using the user name, so I am using the UID & GID
# set in the base ros-cuda image.
USER 1001:1001

# Copy our custom packages to the workspace:
# COPY ./src/ansr_hello_world_py /home/performer/dev_ws/src/ansr_hello_world_py
# Download our model weights so we can burn them into the docker image.
# RUN ["/bin/bash", "-c", "python3 /home/performer/dev_ws/src/ansr_hello_world_py/scripts/download_weights.py"]

# Add additional packages here!
COPY ./verifiable-compositional-rl /home/performer/dev_ws/src/verifiable-compositional-rl
COPY ./mission-schema /home/performer/dev_ws/src/mission-schema
COPY ./submission_info /submission_info

# Build all the packages
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && colcon build --symlink-install"]

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["/run_node.sh"]
