FROM nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04

# 1) Install dependencies
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update
RUN apt-get install -y gcc g++ vim git curl sudo dialog apt-utils
# 2) Install python 3.7 (it requires also python 3.6 so it will be in the docker under python3.6)
RUN yes | sudo apt-get install build-essential libpq-dev libssl-dev openssl libffi-dev zlib1g-dev libssl1.1 openssl
RUN sudo apt-get install python3-pip python3.7-dev dbus dh-python gir1.2-glib-2.0 libapparmor1 libdbus-1-3 libexpat1-dev libgirepository-1.0-1 libglib2.0-0 libglib2.0-data libicu60 libpython3-dev libpython3-stdlib libpython3.6-dev libpython3.7 libpython3.7-dev libpython3.7-minimal libpython3.7-stdlib libxml2 python-pip-whl python3 python3-asn1crypto python3-cffi-backend python3-crypto python3-cryptography python3-dbus python3-dev python3-distutils python3-gi python3-idna python3-keyring python3-keyrings.alt python3-lib2to3 python3-minimal python3-pip python3-pkg-resources python3-secretstorage python3-setuptools python3-six python3-wheel python3-xdg python3.6 python3.6-dev python3.6-minimal python3.7 python3.7-dev python3.7-minimal shared-mime-info xdg-user-dirs
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2
RUN ln -sv /usr/bin/python3 /usr/bin/python
RUN ln -sv /usr/bin/pip3 /usr/bin/pip
RUN pip install --upgrade pip
RUN sudo apt-get update
RUN yes | sudo apt install libpython-all-dev python2.7 python2.7-dev python-pip
ARG USER_ID=1000
RUN useradd -m --no-log-init --system  --uid ${USER_ID} dls_user -g sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER dls_user
WORKDIR /home/dls_user/obj_dect_T171Cu102_ws_home
ENV PATH="/home/dls_user/.local/bin:${PATH}"
# 3) Install OpenCV and its dependencies
RUN sudo apt-get install libjpeg-dev libjpeg-turbo8 libjpeg-turbo8-dev libjpeg8 libjpeg8-dev
RUN yes | sudo apt-get install python3-opencv 
RUN yes | sudo apt-get install libcanberra-gtk-module
RUN pip install opencv-python scikit-learn scipy matplotlib pip
RUN pip install --user cmake cython pillow
# 4) Install pytorch and torchvision 
# RUN pip install --user pip install torch==1.7.1 torchvision==0.8.2 torchaudio==0.7.2
RUN pip install torch==1.7.1 torchvision==0.8.2 torchaudio==0.7.2
RUN pip3 install pandas requests tqdm seaborn
# 5) Install ...
# 6) Install ...
# 7) Install Realsense
RUN sudo apt-get install software-properties-common cron distro-info-data iso-codes lsb-release powermgmt-base python-apt-common python3-apt python3-software-properties software-properties-common unattended-upgrades
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 2
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1
RUN sudo apt-key adv --no-tty --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --no-tty --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN sudo apt-get update
RUN yes | sudo apt-get install librealsense2-dkms
RUN yes | sudo apt-get install librealsense2-utils
RUN sudo apt-get install librealsense2-dev
RUN sudo apt-get install librealsense2-dbg
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2
RUN pip install pyrealsense2
RUN python2.7 -m pip install pyrealsense2
RUN pip2 install --upgrade pip
RUN pip2 install opencv-python==4.2.0.32
# 8) ROS  without any graphical interface
RUN sudo apt-get install -y lsb-release
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt install ros-melodic-desktop-full -y
# RUN source ~/.bashrc
RUN sudo apt-get install -y python3-catkin-pkg-modules python3-rospkg-modules ros-melodic-realsense2-camera ros-melodic-realsense2-description
RUN sudo apt-get install -y ros-melodic-cv-bridge ros-melodic-vision-opencv
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
RUN sudo apt-get install -y nano   
RUN sudo apt-get install -y python-catkin-tools
RUN pip install --user rospkg catkin_pkg
# Install cv_bridge for python3.7
RUN mkdir --parents /home/dls_user/cv_bridge_ws/src
WORKDIR /home/dls_user/cv_bridge_ws
RUN catkin config \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.7 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.7m.so
RUN catkin config --install
RUN git -C /home/dls_user/cv_bridge_ws/src clone -b 1.12.8 https://github.com/ros-perception/vision_opencv.git 
RUN catkin config --extend /opt/ros/melodic && \
    catkin clean --yes && \
    catkin build
RUN sudo apt-get install -y python-wstool
RUN sudo apt-get install -y python-rosdep && sudo rosdep init 
RUN rosdep update
# RUN pip uninstall em && pip install --user empy
# RUN  . /opt/ros/melodic/setup.sh && mkdir -p geometry2_ws/src && cd geometry2_ws && catkin_make && . devel/setup.sh \
# &&  wstool init && wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel \
# && wstool up && rosdep install --from-paths src --ignore-src -y -r && catkin_make install --cmake-args \
#             -DCMAKE_BUILD_TYPE=Release \
#             -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#             -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
#             -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# RUN mv ~/geometry2_ws/install ~/.local/install_dir/install_geometry2 && sudo rm -rf geometry2_ws && sudo rm -rf /opt/ros/melodic/lib/python2.7/dist-packages/tf2*
# RUN echo "source ~/.local/install_dir/install_geometry2/setup.bash --extend" >> ~/.bashrc
# RUN pip install --user scipy
WORKDIR /home/dls_user
# Install pascal VOC generator
RUN pip3 install pascal-voc-writer
