FROM nvidia/cuda:12.8.0-cudnn-devel-ubuntu24.04

# Install python
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
     python3.12 \
     python3.12-venv \
     python3.12-dev \
     python3-pip \
 && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3.12 /usr/bin/python3 \
 # optional: expose `python` alias pointing to python3.12
 && ln -sf /usr/bin/python3.12 /usr/bin/python

# pip install requirements.txt
WORKDIR /workspace
COPY ../requirements.txt .
RUN python3 -m venv /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN /opt/venv/bin/pip install \
    --no-cache-dir -r requirements.txt

# Gazebo---
# https://gazebosim.org/docs/harmonic/install_ubuntu/
RUN apt-get update\
    && apt-get install -y curl lsb-release gnupg

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null\
    && apt-get update\
    && apt-get install -y gz-harmonic

#ROS2 JAZZY----
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# install locals
RUN apt update && apt install -y locales --no-install-recommends\
    && locale-gen en_US en_US.UTF-8\
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8\
    && LANG=en_US.UTF-8

#ensure ubutnu universe repository is enabled 
RUN apt install -y software-properties-common\
    && add-apt-repository universe

RUN apt-get update && apt-get install -y\
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && UBUNTU_CODENAME=$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}}) \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y \
    ros-dev-tools

RUN apt update && apt upgrade -y && apt install -y \
    ros-jazzy-desktop