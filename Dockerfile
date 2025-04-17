# parameters
ARG REPO_NAME="<gesture_robot>"
ARG DESCRIPTION="<Final Project for ME740 with Duckiebot>"
ARG MAINTAINER="<Yunda Li> (<yunli1618@gmail.com>)"
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG ICON="cube"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO=daffy
ARG DOCKER_REGISTRY=docker.io
ARG BASE_IMAGE=dt-ros-commons
ARG BASE_TAG=${DISTRO}-${ARCH}
#ARG BASE_TAG=${DISTRO}-ubuntu18.04-${ARCH}

ARG LAUNCHER=default

# define base image
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as base

# recall all arguments
ARG DISTRO
ARG REPO_NAME
ARG DESCRIPTION
ARG MAINTAINER
ARG ICON
ARG BASE_TAG
ARG BASE_IMAGE
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT

# check build arguments
RUN dt-build-env-check "${REPO_NAME}" "${MAINTAINER}" "${DESCRIPTION}"

# define/create repository path
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
RUN mkdir -p "${REPO_PATH}" "${LAUNCH_PATH}"
WORKDIR "${REPO_PATH}"

# keep some arguments as environment variables
ENV DT_MODULE_TYPE="${REPO_NAME}" \
    DT_MODULE_DESCRIPTION="${DESCRIPTION}" \
    DT_MODULE_ICON="${ICON}" \
    DT_MAINTAINER="${MAINTAINER}" \
    DT_REPO_PATH="${REPO_PATH}" \
    DT_LAUNCH_PATH="${LAUNCH_PATH}" \
    DT_LAUNCHER="${LAUNCHER}"

# install apt dependencies
COPY ./dependencies-apt.txt "${REPO_PATH}/"
RUN dt-apt-install ${REPO_PATH}/dependencies-apt.txt

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${REPO_PATH}/"
RUN dt-pip3-install "${REPO_PATH}/dependencies-py3.*"

#MAYBE ADD INSTALLATION STUFF HERE?=======================


# Add NVIDIA package repositories
RUN apt-get clean

RUN apt-get update && apt-get install -y python3-tk  
# # Add NVIDIA package repositories
# RUN apt-get update && apt-get install -y wget gnupg2 software-properties-common
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
# RUN mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
# # Download and install the CUDA 12.1 repository package for Ubuntu 22.04
# RUN wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb
# RUN dpkg -i cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb

# # Alternatively, if the URL does not work, manually add the key like this:
# RUN cp /var/cuda-repo-ubuntu2204-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/

RUN apt-get update

# RUN apt-get clean



# Install CUDA toolkit 12.1
# RUN apt-get install -y cuda-toolkit-12-1

# Install NVIDIA TensorRT WHYYYYYY
# RUN os="ubuntu2004"
# RUN tag="10.0.0-cuda-12.1"
# RUN dpkg -i nv-tensorrt-local-repo-${os}-${tag}_1.0-1_amd64.deb
# RUN cp /var/nv-tensorrt-local-repo-${os}-${tag}/*-keyring.gpg /usr/share/keyrings/
# RUN apt-get update

# RUN python3 -m pip install --upgrade pip
# RUN python3 -m pip install wheel
# RUN python3 -m pip install --pre --upgrade tensorrt



# RUN apt-get install -y tensorrt
# RUN apt-get install -y python3-libnvinfer-dev

# Set CUDA_HOME environment variable
ENV CUDA_HOME=/usr/local/cuda-12.1
ENV PATH=$PATH:/usr/local/cuda-12.1/bin
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-12.1/lib64

# RUN pip install mediapipe

# #Install Git, clone torch2trt from GitHub, and install required Python packages
# RUN apt-get update && apt-get install -y git
# RUN git clone https://github.com/NVIDIA-AI-IOT/torch2trt /torch2trt
# RUN pip3 install packaging  # Install the missing 'packaging' module
# RUN cd /torch2trt && python3 setup.py install

# #Clone trtpose repository
# RUN git clone https://github.com/NVIDIA-AI-IOT/trt_pose /trt_pose
# RUN cd /trt_pose && python3 setup.py install

#====================END INSTALL STUFF ===================


# copy the source code
COPY ./packages "${REPO_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.module.type="${REPO_NAME}" \
    org.duckietown.label.module.description="${DESCRIPTION}" \
    org.duckietown.label.module.icon="${ICON}" \
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"
# <== Do not change the code above this line
# <==================================================

#RUN apt-get clean

# Add NVIDIA package repositories
#RUN apt-get update && apt-get install -y wget gnupg2 software-properties-common
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
#RUN mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
#RUN wget http://developer.download.nvidia.com/compute/cuda/11.1.1/local_installers/cuda-repo-ubuntu2004-11-1-local_11.1.1-455.32.00-1_amd64.deb
#RUN dpkg -i cuda-repo-ubuntu2004-11-1-local_11.1.1-455.32.00-1_amd64.deb
#RUN apt-key add /var/cuda-repo-ubuntu2004-11-1-local/7fa2af80.pub
#RUN apt-get update

#RUN apt-get clean

# Install CUDA toolkit 11.1
#RUN apt-get install -y cuda-toolkit-11-1

# Set CUDA_HOME environment variable
#ENV CUDA_HOME=/usr/local/cuda-11.1
#ENV PATH=$PATH:/usr/local/cuda-11.1/bin
#ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.1/lib64

# Install Git, clone torch2trt from GitHub, and install required Python packages
#RUN apt-get update && apt-get install -y git
#RUN git clone https://github.com/NVIDIA-AI-IOT/torch2trt /torch2trt
#RUN pip3 install packaging  # Install the missing 'packaging' module
#RUN cd /torch2trt && python3 setup.py install

# Clone trtpose repository
#RUN git clone https://github.com/NVIDIA-AI-IOT/trt_pose /trt_pose
#RUN cd /trt_pose && python3 setup.py install




