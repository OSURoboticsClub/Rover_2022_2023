FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y \
	ros-foxy-image-common ros-foxy-image-pipeline ros-foxy-image-transport-plugins \
	ros-foxy-laser-geometry ros-foxy-perception-pcl ros-foxy-vision-opencv ros-foxy-ros-ign \
	ros-foxy-nmea-msgs xauth nano socat xxd pip

RUN pip install numpy pyserial pynmea2 minimalmodbus

RUN /bin/bash -c 'echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc'

RUN ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N '' <<< y

ENV DISPLAY=host.docker.internal:0.0

CMD ["bash"]